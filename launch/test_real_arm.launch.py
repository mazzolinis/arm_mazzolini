from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, RegisterEventHandler, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, Command
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

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
    # nodes.append(initial_state_broadcaster_spawner_stopped)

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

    activate_controller = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/omni_controller/activate_srv',
            'std_srvs/srv/SetBool',
            '{data: true}'
        ] 
    )
    # nodes.append(activate_controller)

    # Terzo avvio il nodo di controllo del braccio
    real_arm_test_node = Node(
        package="arm_mazzolini",
        executable="real_arm_test_node",
        output="screen",
        # arguments = [controller_config_path]
    )
    nodes.append(RegisterEventHandler(
        OnProcessExit(
            target_action=activate_controller,
            on_exit=[real_arm_test_node]
        )
    ))
    nodes.append(real_arm_test_node)

    nuc_heartbeat_node = Node(
        package="arm_mazzolini",
        executable="nuc_heartbeat_node",
        name="nuc_heartbeat_node"
    )
    nodes.append(nuc_heartbeat_node)


    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [pkg_share, "urdf", "weeder_robot_real.xacro"]
            ),
            " ",
            "is_light:=", "true",
            " ",
            "controller_yaml:=", " ",
        ]
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": ParameterValue(robot_description_content, value_type=str)}
        ],
        # remappings=[
        #     ('/omni_controller/joints_state', '/joint_states')  # redirige il topic
        # ]
    )
    nodes.append(robot_state_publisher)

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

    return LaunchDescription(nodes)
