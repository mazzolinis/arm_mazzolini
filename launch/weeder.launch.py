from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription, RegisterEventHandler, TimerAction, SetEnvironmentVariable
from launch.conditions import IfCondition,UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command,FindExecutable,LaunchConfiguration,PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_real_hw",
            default_value="false",
            description="Use real arm if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "height",
            default_value="0.4",
            description="Height to spawn the robot at",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_height",
            default_value="0.4",
            description="Height of barn relative to flat map",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "is_light",
            default_value="true",
            description="Uses IPG Photonics laser head if true, uses Futonics otherwise"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Set to false to disable RViz",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_camera",
            default_value="true",
            description="Set true to use simulated camera",
    )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_real_hw = LaunchConfiguration("use_real_hw")
    height = LaunchConfiguration("height")
    world_height = LaunchConfiguration("world_height")
    is_light = LaunchConfiguration("is_light")
    use_rviz = LaunchConfiguration("use_rviz")
    use_camera = LaunchConfiguration("use_camera")

    # Other parameters (this can be set only from launch file)
    pkg_share = FindPackageShare("arm_mazzolini")
    robot_name = "weeder_robot"
    controller = PythonExpression([
        "'light_controller.yaml' if '", is_light, "' == 'true' else 'heavy_controller.yaml'"   # This works
    ])
    controller_config_file = PathJoinSubstitution(
        [pkg_share, "config", controller],
    )
    parameters_file = PathJoinSubstitution([pkg_share, "config", "weeder_parameters.yaml"])
    camera_config_file = PathJoinSubstitution([pkg_share, "config", "simulated_D435_parameters.yaml"])
    output_file = PathJoinSubstitution(["/home", "simone", "Scrivania", "first_test_data.csv"])

    # Path to robot description file (xacro)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [pkg_share, "urdf", "weeder_robot.xacro"]
            ),
            " ",
            "is_light:=", is_light,
            " ",
            "use_camera:=", use_camera,
            " ",
            "controller_yaml:=", controller_config_file,
            " ",
            "use_sim_time:=", use_sim_time
        ]
    )

    # Launch Gazebo
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                # [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                [FindPackageShare("arm_mazzolini"), "launch", "gz_sim_silent.launch.py"] # replaced to avoid print at every message sent by controller
            )
        ),
        launch_arguments={
            # 'gz_args': 'default.sdf',
            # 'gz_args': PathJoinSubstitution([FindPackageShare("arm_mazzolini"), "worlds", "agricultural_world.sdf"]),
            'gz_args': PathJoinSubstitution([FindPackageShare("arm_mazzolini"), "worlds", "virtual_maize_field.sdf"]),
            'use_sim_time': use_sim_time,
            "verbose": "false",
        }.items(),
        condition = IfCondition(use_sim_time),
    )
    gazebo = []
    # gazebo.append(SetEnvironmentVariable(
    #     name='RCUTILS_LOGGING_SEVERITY_THRESHOLD',
    #     value='30'   # 10=DEBUG, 20=INFO, 30=WARN, 40=ERROR, 50=FATAL
    # ))
    gazebo.append(gz_launch)

    # Nodes
    nodes = []

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": ParameterValue(robot_description_content, value_type=str)},
            {"use_sim_time": use_sim_time}
        ]
    )
    nodes.append(robot_state_publisher)

    spawn_entity = Node(
        package = "ros_gz_sim",
        executable = "create",
        arguments = ['-topic', '/robot_description',
                    '-name', robot_name,
                    '-x', '0', '-y', '0.55', '-z', PythonExpression([height, ' + ', world_height]),
                    '-R', '0', '-P', '0', '-Y', '-3.14159',],
        parameters=[{'use_sim_time': use_sim_time}],
        output = 'screen',
        condition = IfCondition(use_sim_time),
    )
    nodes.append(spawn_entity)
    # nodes.append(TimerAction(period=5.0, actions=[spawn_entity]))

    # spawn_world = Node(
    #     package = "ros_gz_sim",
    #     executable = "create",
    #     arguments = ['-file', PathJoinSubstitution([FindPackageShare("arm_mazzolini"), "urdf", "agriculture_geometry.urdf"]),
    #                  '-name', 'agriculture_world',
    #                  '-x', '0', '-y', '-4.0', '-z', PythonExpression([world_height, '+ 0.1']),
    #                  '-R', '0', '-P', '0', '-Y', '0',],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output = 'screen',
    #     condition = IfCondition(use_sim_time),
    # )
    # nodes.append(spawn_world)

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("arm_mazzolini"), "rviz", "second_config.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )
    nodes.append(rviz_node)

    # gz_bridge=Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=[
    #     '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
    #     '/world/default/create@ros_gz_interfaces/srv/SpawnEntity',
    #     '/world/default/remove@ros_gz_interfaces/srv/DeleteEntity',
    #     # '/simulated_D435/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo', # Should I use it?
    #     '/simulated_D435/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
    #     '/simulated_D435/image@sensor_msgs/msg/Image[gz.msgs.Image',
    #     # '/model/weeder_robot/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
    #     '/world/default/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
    #     ],
    #     condition=IfCondition(use_sim_time),
    #     output='screen'
    # )
    # nodes.append(gz_bridge)

    gz_bridge=Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        '/world/agricultural_world/create@ros_gz_interfaces/srv/SpawnEntity',
        '/world/agricultural_world/remove@ros_gz_interfaces/srv/DeleteEntity',
        # '/simulated_D435/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo', # Should I use it?
        '/simulated_D435/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/simulated_D435/image@sensor_msgs/msg/Image[gz.msgs.Image',
        # '/model/weeder_robot/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
        '/world/agricultural_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        condition=IfCondition(use_sim_time),
        output='screen'
    )
    nodes.append(gz_bridge)


    gazebo_target_visualizer = Node(
        package = "arm_mazzolini",
        executable = "gazebo_target_visualizer",
        name = "gazebo_target_visualizer",
        parameters = [{"use_sim_time":use_sim_time},
                      parameters_file],
        output = "screen",
        condition = IfCondition(use_sim_time)
    )
    nodes.append(gazebo_target_visualizer)

    # rviz_target_visualizer = Node(
    #     package = "arm_mazzolini",
    #     executable = "rviz_target_visualizer",
    #     name = "rviz_target_visualizer",
    #     parameters = [{"use_sim_time":use_sim_time}],
    #     output = "screen",
    #     condition = IfCondition(use_rviz)
    # )
    # nodes.append(RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[TimerAction(period=20.0, actions=[rviz_target_visualizer])])))

    # This node spawns controllers (diff drive, joint trajectory), weeder controller and target spawner when simulation starts
    simulation_spawner = Node(
        package='arm_mazzolini',
        executable='simulation_process_spawner.py',
        name='simulation_process_spawner',
        parameters=[{
            'controller_yaml': controller_config_file,
            'parameters_yaml': parameters_file,
            'world_height': world_height,
            'use_sim_time': use_sim_time,
            'use_real_hw': use_real_hw,
            'output_file': output_file,
        }],
        condition = IfCondition(use_sim_time), # This node works only in simulation
        output='screen',
    )
    nodes.append(simulation_spawner)

    camera_info_publisher = Node(
        package="arm_mazzolini",
        executable="camera_info_publisher",
        name="camera_info_publisher",
        parameters=[
            {"camera_name": "simulated_D435"}, # TODO: change this, it has to match the name in the yaml file, not a good practice
            {"camera_config_file": camera_config_file},
            {"image_topic": "/simulated_D435/image"},
            {"camera_info_topic": "/simulated_D435/camera_info"},
            {"use_sim_time": use_sim_time},
        ],
        condition = IfCondition(
            PythonExpression(["'", use_sim_time, "' == 'true' and '", use_camera, "' == 'true'"])
        ),
    )
    nodes.append(camera_info_publisher)

    rgb_rectify = Node(
        package="image_proc",
        executable="rectify_node",
        namespace="simulated_D435",
        name="rgb_rectify_node",
        remappings=[
            ("image", "/simulated_D435/image"),
            ("camera_info", "/simulated_D435/camera_info"), # TODO: change names to parameters, less space for error
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition = IfCondition(
            PythonExpression(["'", use_sim_time, "' == 'true' and '", use_camera, "' == 'true'"])
        )
    )
    nodes.append(rgb_rectify)

    # TODO: depth/rectify non esiste. Devo usare qualcos'altro oppure è automatico?
    # depth_rectify = Node(
    #     package="depth_image_proc",
    #     executable="rectify",
    #     namespace="simulated_D435",
    #     remappings=[
    #         ("image", "/simulated_D435/depth_image"),
    #         ("camera_info", "/simulated_D435/camera_info"),
    #     ],
    #     parameters=[{"use_sim_time": use_sim_time}],
    #     condition = IfCondition(
    #         PythonExpression(["'", use_sim_time, "' == 'true' and '", use_camera, "' == 'true'"])
    #     )
    # )
    # nodes.append(depth_rectify)


    # # Create point cloud from depth image (is this necessary?)
    # point_cloud_node = Node(
    #     package="depth_image_proc",
    #     executable="point_cloud_xyzrgb_node",
    #     namespace="simulated_D435",
    #     remappings=[
    #         ("rgb/image_rect", "/simulated_D435/image_rect"),
    #         ("rgb/camera_info", "/simulated_D435/camera_info"),
    #         ("depth/image", "/simulated_D435/depth_image"),
    #         ("points", "/simulated_D435/points"),
    #     ],
    #     parameters=[{"use_sim_time": use_sim_time}],
    #     condition = IfCondition(
    #         PythonExpression(["'", use_sim_time, "' == 'true' and '", use_camera, "' == 'true'"])
    #     )
    # )
    # nodes.append(point_cloud_node)

    weeder_controller = Node(
        package="arm_mazzolini",
        executable="weeder_controller",
        name="weeder_controller",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"use_real_hw": use_real_hw},
                    controller_config_file],
        condition = UnlessCondition(use_sim_time)
    )
    nodes.append(weeder_controller)


    return LaunchDescription(declared_arguments + gazebo + nodes)
