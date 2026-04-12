from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, RegisterEventHandler, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description(): 

    pkg_share = FindPackageShare("arm_mazzolini")
    controller_config_path = PathJoinSubstitution([pkg_share, "config", "real_arm_controller.yaml"])

    nodes = []
    # Per prima cosa avvio lo state_controller
    initial_state_broadcaster_spawner_stopped = Node(
    package="controller_manager",
        executable="spawner",
        arguments=["state_broadcaster", "-c", "/controller_manager"],    
    )
    nodes.append(initial_state_broadcaster_spawner_stopped)

    # Secondo avvio il nodo controller
    initial_joint_controller_spawner_stopped = Node(
    package="controller_manager",
        executable="spawner",
        arguments=["joint_controller", "-c", "/controller_manager"],
        
    )
    nodes.append(RegisterEventHandler(
        OnProcessExit(
            target_action=initial_state_broadcaster_spawner_stopped,
            on_exit=[initial_joint_controller_spawner_stopped]
        )
    ))
    # nodes.append(initial_joint_controller_spawner_stopped)

    # Terzo avvio il nodo di controllo del braccio
    real_arm_test_node = Node(
        package="arm_mazzolini",
        executable="real_arm_test_node",
        output="screen",
        # arguments = [controller_config_path]
    )
    nodes.append(RegisterEventHandler(
        OnProcessExit(
            target_action=initial_joint_controller_spawner_stopped,
            on_exit=[real_arm_test_node]
        )
    ))
    # nodes.append(real_arm_test_node)
    
    return LaunchDescription(nodes)
