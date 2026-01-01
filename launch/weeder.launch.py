# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription, RegisterEventHandler, TimerAction
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
            default_value="2.0",
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
    height = LaunchConfiguration("height")
    world_height = LaunchConfiguration("world_height")
    is_light = LaunchConfiguration("is_light")
    use_rviz = LaunchConfiguration("use_rviz")
    use_camera = LaunchConfiguration("use_camera")

    pkg_share = FindPackageShare("arm_mazzolini")

    robot_name = "weeder_robot"
    
    # ================================================
    # TODO: Find a way to select controller at runtime
    # ================================================

    # Select controller at runtime (different PD gains for each arm)
    # if is_light == 'true':
    #     controller = 'weeder_controller_light.yaml'
    # else:
    #     controller = 'weeder_controller_heavy.yaml'
    
    controller = PythonExpression([
        "'weeder_controller_light.yaml' if '", is_light, "' == 'true' else 'weeder_controller_heavy.yaml'"
    ])

    # controller = 'weeder_controller_light.yaml'  # For now, only light version is used
    
    controller_config_file = PathJoinSubstitution(
        [pkg_share, "config", controller],
    )

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
            "use_sime_time:=", use_sim_time
        ]
    )
    
    # Launch Gazebo
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            # 'gz_args': 'default.sdf',
            'gz_args': PathJoinSubstitution([FindPackageShare("arm_mazzolini"), "worlds", "agricultural_world.sdf"]),
            # 'gz_args': 'camera_sensor.sdf',
            'use_sim_time': use_sim_time,
            "verbose": "true",
        }.items(),
    )

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
                    '-x', '0', '-y', '0', '-z', PythonExpression([height, ' + ', world_height]),
                    '-R', '0', '-P', '0', '-Y', '0',],
        parameters=[{'use_sim_time': use_sim_time}],
        output = 'screen',
    )
    nodes.append(spawn_entity)
    # nodes.append(TimerAction(period=5.0, actions=[spawn_entity]))

    spawn_world = Node(
        package = "ros_gz_sim",
        executable = "create",
        arguments = ['-file', PathJoinSubstitution([FindPackageShare("arm_mazzolini"), "urdf", "agriculture_geometry.urdf"]),
                     '-name', 'agriculture_world',
                     '-x', '0', '-y', '0', '-z', world_height,
                     '-R', '0', '-P', '0', '-Y', '0',],
        parameters=[{'use_sim_time': use_sim_time}],
        output = 'screen',
    )
    nodes.append(spawn_world)

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
        '/world/default/create@ros_gz_interfaces/srv/SpawnEntity',
        '/world/default/remove@ros_gz_interfaces/srv/DeleteEntity',
        # left/right images
        # '/camera/left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
        # '/camera/right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
        # # camera_info 
        # # '/camera/left/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        # # '/camera/right/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        # # depth & points
        # '/camera/depth/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
        # '/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
        condition=IfCondition(use_sim_time),
        output='screen'
    )
    nodes.append(gz_bridge)

    gazebo_target_visualizer = Node(
        package = "arm_mazzolini",
        executable = "gazebo_target_visualizer",
        name = "gazebo_target_visualizer",
        parameters = [{"use_sim_time":use_sim_time}],
        output = "screen"
    )
    nodes.append(RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[TimerAction(period=20.0, actions=[gazebo_target_visualizer])])))

    # rviz_target_visualizer = Node(
    #     package = "arm_mazzolini",
    #     executable = "rviz_target_visualizer",
    #     name = "rviz_target_visualizer",
    #     parameters = [{"use_sim_time":use_sim_time}],
    #     output = "screen",
    #     condition = IfCondition(use_rviz)
    # )
    # nodes.append(RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[TimerAction(period=20.0, actions=[rviz_target_visualizer])])))

    # camera_info_left = Node(
    #     package='arm_mazzolini',
    #     executable='camera_info_publisher',
    #     name='camera_info_left',
    #     parameters=[{
    #         'camera_name': 'realsense_d435_left',
    #         'camera_info_url': PathJoinSubstitution([pkg_share, 'config', 'camera_info_left.yaml']),
    #         'image_topic': '/camera/left/image_raw',
    #         'camera_info_topic': '/camera/left/camera_info',
    #         'use_sim_time': use_sim_time,
    #     }],
    #     condition=IfCondition(use_camera),
    #     output='screen',
    # )
    # nodes.append(camera_info_left)

    # camera_info_right = Node(
    #     package='arm_mazzolini',
    #     executable='camera_info_publisher',
    #     name='camera_info_right',
    #     parameters=[{
    #         'camera_name': 'realsense_d435_right',
    #         'camera_info_url': PathJoinSubstitution([pkg_share, 'config', 'camera_info_right.yaml']),
    #         'image_topic': '/camera/right/image_raw',
    #         'camera_info_topic': '/camera/right/camera_info',
    #         'use_sim_time': use_sim_time,
    #     }],
    #     condition=IfCondition(use_camera),
    #     output='screen',
    # )
    # nodes.append(camera_info_right)

    # This node spawns controllers (diff drive, joint trajectory), kinematic node and target spawner when simulation starts
    simulation_spawner = Node(
        package='arm_mazzolini',
        executable='simulation_process_spawner.py',
        name='simulation_process_spawner',
        parameters=[{
            'controller_yaml': controller_config_file,
            'world_height': world_height,
            'use_sim_time': use_sim_time,
        }],
        condition = IfCondition(use_sim_time), # This node works only in simulation
        output='screen',
    )
    nodes.append(simulation_spawner)


    return LaunchDescription(declared_arguments + [gz_launch] + nodes)
