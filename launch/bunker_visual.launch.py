from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
)
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "world",
            default_value="default.sdf",
            description="Gazebo world file name",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        ),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    pkg_share = get_package_share_directory("arm_mazzolini")

    # Paths to files
    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "bunker_control.xacro"])
    controller_config_file = PathJoinSubstitution([pkg_share, "config", "bunker_fake.yaml"])
    world_file = PathJoinSubstitution([pkg_share, "world", "custom_world.sdf"])

    robot_description_content = Command(["xacro ", xacro_file])

    gz_sim_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("ros_gz_sim"),
            "launch", "gz_sim.launch.py",
        ]),
        
        launch_arguments={
            'gz_args': ['default.sdf'],
            'use_sim_time':use_sim_time,
            "verbose":"true",
            'on_exit_shutdown':'true', # Important to shutdown ROS2 when Gazebo is closed
            }.items(),
    )

    # Spawn robot into Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "bunker",
            "-x", "0", "-y", "0", "-z", "0.4",
            "-topic", "/robot_description",
        ],
        output="screen",
    )

    # Publish TF & robot state
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            'use_sim_time':use_sim_time,
            'robot_description':robot_description_content}],
        output="screen",
    )

    controller_manager = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content,
            },
            controller_config_file],
        output='screen'
    )

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controller_config_file,
        ],
        output='screen'
    )
    
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--param-file', controller_config_file,
        ],
        output='screen'
    )

    clock_bridge=Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription(
        declared_arguments
        + [
            gz_sim_launch,
            robot_state_publisher,
            spawn_entity,
            clock_bridge,
            # RegisterEventHandler(OnProcessExit(target_action=spawn_entity,on_exit=[controller_manager])),
            RegisterEventHandler(OnProcessExit(target_action=spawn_entity,on_exit=[TimerAction(period=3.0, actions=[joint_state_broadcaster])])),            
            RegisterEventHandler(OnProcessExit(target_action=spawn_entity,on_exit=[TimerAction(period=3.0, actions=[diff_drive_controller])])),
        ]
    )
