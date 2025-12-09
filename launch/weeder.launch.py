import os
# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription, RegisterEventHandler, TimerAction, SetEnvironmentVariable
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
            "use_gui", # TODO: remove this argument, simulation can't use joint_trajectory_gui anymore
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
            "is_wheeled", # TODO: remove this argument too
            default_value="true",
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

    pkg_share = FindPackageShare("arm_mazzolini")

    robot_name = "weeder_robot"
    controller_config_file = PathJoinSubstitution(
        [pkg_share, "config", controller],
    )
    set_ign_resource = SetEnvironmentVariable(
        name = "IGN_GAZEBO_RESOURCE_PATH",
        value = [os.getenv("IGN_GAZEBO_RESOURCE_PATH", ""), ":" , PathJoinSubstitution([pkg_share, "models"]), ":", PathJoinSubstitution([pkg_share, "worlds"])]
    )
    set_display = SetEnvironmentVariable(
        name='DISPLAY',
        value=os.getenv('DISPLAY', ':0')
    )
    set_ign_log = SetEnvironmentVariable(
        name='IGN_LOG_LEVEL',
        value=os.getenv('IGN_LOG_LEVEL', '4') 
    )
    sets = [set_ign_resource, set_display, set_ign_log] # Add these in LaunchDescription if you want to use agricultural_world_with_barn.sdf

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [pkg_share, "urdf", "weeder_robot.xacro"]
            ),
            " ",
            "is_wheeled:=", is_wheeled,
            " ",
            "is_light:=", is_light,
            " ",
            "controller_yaml:=", controller_config_file,
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
            # 'gz_args': PathJoinSubstitution([FindPackageShare("arm_mazzolini"), "worlds", "agricultural_world_with_barn.sdf"]),
            'use_sim_time': use_sim_time,
            "verbose": "true",
        }.items(),
    )

    # Nodes
    nodes = []

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time}
        ]
    )
    nodes.append(robot_state_publisher)

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

    spawn_world = Node(
        package = "ros_gz_sim",
        executable = "create",
        arguments = ['-file', PathJoinSubstitution([FindPackageShare("arm_mazzolini"), "urdf", "agriculture_geometry.urdf"]),
                     '-name', 'agriculture_world',
                     '-x', '0', '-y', '0', '-z', '0',
                     '-R', '0', '-P', '0', '-Y', '0',],
        parameters=[{'use_sim_time': use_sim_time}],
        output = 'screen',
    )
    nodes.append(spawn_world)

    joint_state_pub_gui = Node(
        package = "joint_state_publisher_gui",
        executable = "joint_state_publisher_gui",
        name = "joint_state_publisher_gui",
        output = "screen",
        condition = IfCondition(use_gui),
        parameters=[{"use_sim_time": use_sim_time}],
    )
    nodes.append(joint_state_pub_gui)

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

    clock_bridge=Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        condition=IfCondition(use_sim_time),
        output='screen'
    )
    nodes.append(clock_bridge)

    kinematic_node = Node(
        package = "arm_mazzolini",
        executable = "kinematic_node",
        name = "kinematic_node",
        parameters = [{"use_sim_time":use_sim_time}, controller_config_file],
        output = "screen",
        condition = UnlessCondition(use_gui)
    )
    nodes.append(RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[TimerAction(period=15.0, actions=[kinematic_node])])))

    target_spawner = Node(
        package = "arm_mazzolini",
        executable = "target_spawner",
        name = "target_spawner",
        parameters = [{"use_sim_time":use_sim_time}],
        output = "screen",
        condition = UnlessCondition(use_gui)
    )
    nodes.append(RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[TimerAction(period=20.0, actions=[target_spawner])])))

    # gazebo_target_visualizer = Node(
    #     package = "arm_mazzolini",
    #     executable = "gazebo_target_visualizer",
    #     name = "gazebo_target_visualizer",
    #     parameters = [{"use_sim_time":use_sim_time}],
    #     output = "screen",
    #     condition = UnlessCondition(use_gui)
    # )
    # nodes.append(RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[TimerAction(period=20.0, actions=[gazebo_target_visualizer])])))

    # rviz_target_visualizer = Node(
    #     package = "arm_mazzolini",
    #     executable = "rviz_target_visualizer",
    #     name = "rviz_target_visualizer",
    #     parameters = [{"use_sim_time":use_sim_time}],
    #     output = "screen",
    #     condition = IfCondition(use_rviz)
    # )
    # nodes.append(RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[TimerAction(period=20.0, actions=[rviz_target_visualizer])])))

    return LaunchDescription(declared_arguments + [gz_launch] + nodes)
