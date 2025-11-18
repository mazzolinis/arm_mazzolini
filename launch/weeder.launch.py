# import os
# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition,UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command,FindExecutable,LaunchConfiguration,PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
            "use_gui",
            default_value="false",
            description="Enable gazebo GUI if true",
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
            "is_wheeled",
            default_value="false",
            description="Set true when you'll have wheeled robot defined",
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
            default_value="true",
            description="Set to false to disable RViz",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller",
            default_value = "weeder_controller.yaml",
            description = "Controller configuration file"
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gui = LaunchConfiguration("use_gui")
    height = LaunchConfiguration("height")
    is_wheeled = LaunchConfiguration("is_wheeled")
    is_light = LaunchConfiguration("is_light")
    use_rviz = LaunchConfiguration("use_rviz")
    controller = LaunchConfiguration("controller")

    robot_name = "weeder_robot"
    controller_config_file = PathJoinSubstitution(
        [FindPackageShare("arm_mazzolini"), "config", controller],
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("arm_mazzolini"), "urdf", "weeder_robot.xacro"]
            ),
            " ",
            "is_wheeled:=", is_wheeled,
            " ",
            "is_light:=", is_light,
            " ",
            "controller_yaml:=", controller_config_file,
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    # Launch Gazebo
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            'gz_args': 'default.sdf',
            'use_sim_time': use_sim_time,
            "verbose": "true",
            # "gui": use_gui
            }.items(),
    )

    # Nodes
    nodes = []
    spawn_entity = Node(
        package = "ros_gz_sim",
        executable = "create",
        arguments = ['-topic', '/robot_description',
                    '-name', robot_name,
                    '-x', '0', '-y', '0', '-z', height,
                    '-R', '0', '-P', '0', '-Y', '0',],
        parameters=[{'use_sim_time': use_sim_time}],
        output = 'screen',
    )
    nodes.append(spawn_entity)

    # joint_state_pub = Node(
    #     package = "joint_state_publisher",
    #     executable = "joint_state_publisher",
    #     name = "joint_state_publisher",
    #     output = "screen",
    #     condition = UnlessCondition(use_gui),
    #     parameters=[{"use_sim_time": use_sim_time}],
    # )
    # nodes.append(joint_state_pub)

    joint_state_pub_gui = Node(
        package = "joint_state_publisher_gui",
        executable = "joint_state_publisher_gui",
        name = "joint_state_publisher_gui",
        output = "screen",
        condition = IfCondition(use_gui),
        parameters=[{"use_sim_time": use_sim_time}],
    )
    nodes.append(joint_state_pub_gui)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time}
        ]
    )
    nodes.append(robot_state_publisher)

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("arm_mazzolini"), "rviz", "first_config.rviz"]
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

    joint_state_broadcaster = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = ["joint_state_broadcaster"],
        condition = UnlessCondition(use_gui)
    )
    # nodes.append(joints_state_broadcaster)
    # nodes.append(RegisterEventHandler(OnProcessStart(target_action=controller_manager,on_start=[joint_state_broadcaster])))
    nodes.append(RegisterEventHandler(OnProcessExit(target_action=spawn_entity,on_exit=[TimerAction(period=10.0, actions=[joint_state_broadcaster])])))

    joint_trajectory_controller = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = [
            "joint_trajectory_controller",
            '--controller-manager', '/controller_manager',
            '--param-file', controller_config_file,
            ],
        condition = UnlessCondition(use_gui)
    )
    # nodes.append(RegisterEventHandler(OnProcessStart(target_action=controller_manager,on_start=[joint_trajectory_controller])))
    nodes.append(RegisterEventHandler(OnProcessExit(target_action=spawn_entity,on_exit=[TimerAction(period=10.0, actions=[joint_trajectory_controller])])))

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controller_config_file,
        ],
        condition = IfCondition(is_wheeled),
        output='screen'
    )
    nodes.append(RegisterEventHandler(OnProcessExit(target_action=spawn_entity,on_exit=[TimerAction(period=10.0, actions=[diff_drive_controller])])))

    trajectory_node = Node(
        package = "arm_mazzolini",
        executable = "arm_mazzolini_node",
        name = "arm_mazzolini_node",
        parameters=[{"use_sim_time": use_sim_time}],
        output = "screen",
        condition = UnlessCondition(use_gui)
    )
    # nodes.append(RegisterEventHandler(OnProcessStart(target_action = joint_trajectory_controller, on_start = [trajectory_node])))
    nodes.append(RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[TimerAction(period=20.0, actions=[trajectory_node])])))

    clock_bridge=Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        condition=IfCondition(use_sim_time),
        output='screen'
    )
    nodes.append(clock_bridge)

    return LaunchDescription(declared_arguments + [gz_launch] + nodes)
