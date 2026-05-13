from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription, RegisterEventHandler, TimerAction, ExecuteProcess
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

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    height = LaunchConfiguration("height")
    world_height = LaunchConfiguration("world_height")
    is_light = LaunchConfiguration("is_light")
    use_rviz = LaunchConfiguration("use_rviz")

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
    robot = PythonExpression([
        "'weeder_robot.xacro' if '", use_sim_time, "' == 'true' else 'weeder_robot_real.xacro'" 
    ])
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [pkg_share, "urdf", robot]
            ),
            " ",
            "is_light:=", is_light,
            " ",
            "controller_yaml:=", controller_config_file,
        ]
    )

    # =========================================
    # Gazebo or real camera launch
    # =========================================

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

    real_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        launch_arguments={
            "enable_sync": "true",
            "align_depth.enable": "true",
            "camera_namespace" : "weeder_robot",
            "camera_name" : "real_D455",
        }.items(),
        condition = UnlessCondition(use_sim_time),
    )
    gazebo.append(real_camera_launch)

    # =========================================
    #                 NODES
    # =========================================

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
        '/model/weeder_robot/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
        condition=IfCondition(use_sim_time),
        output='screen'
    )
    nodes.append(gz_bridge)

    ground_truth_tf_publisher = Node(
        package='arm_mazzolini',
        executable='ground_truth_tf_publisher.py',
        name='ground_truth_tf_publisher',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'gz_model_name': 'weeder_robot'},
            {'odom_topic': '/diff_drive_controller/odom'},
        ],
        output='screen',
        condition=IfCondition(use_sim_time),
    )
    nodes.append(ground_truth_tf_publisher)


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
            'output_file': output_file,
        }],
        condition = IfCondition(use_sim_time), # This node works only in simulation
        output='screen',
    )
    nodes.append(simulation_spawner)

    weeder_controller = Node(
        package="arm_mazzolini",
        executable="weeder_controller",
        name="weeder_controller",
        parameters=[
            {"use_sim_time": use_sim_time},
                    parameters_file],
        condition = UnlessCondition(use_sim_time)
    )
    nodes.append(weeder_controller)

    nuc_heartbeat_node = Node(
        package="arm_mazzolini",
        executable="nuc_heartbeat_node",
        name="nuc_heartbeat_node",
        condition=UnlessCondition(use_sim_time)
    )
    nodes.append(nuc_heartbeat_node)

    joints_state_republisher = Node(
        package="arm_mazzolini",
        executable="joints_state_republisher_node",
        name="joints_state_republisher_node",
        output="screen",
        parameters=[
            {"input_topic": "/omni_controller/joints_state"},
            {"output_topic": "/joint_states"},
            {"use_input_stamp": True}
        ],
    )
    nodes.append(joints_state_republisher)


    camera_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_optical_to_realsense_tf",
        arguments=["0", "0", "0", "0", "0", "0",
                   "camera_optical_frame", "real_D455_link"],
        condition=UnlessCondition(use_sim_time),
    )
    nodes.append(camera_tf_publisher)

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
        condition = IfCondition(use_sim_time)
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
        condition = IfCondition(use_sim_time)
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
    #     condition = IfCondition(use_sim_time)
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
    #     condition = IfCondition(use_sim_time)
    # )
    # nodes.append(point_cloud_node)

    return LaunchDescription(declared_arguments + gazebo + nodes)
