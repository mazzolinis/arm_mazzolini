#include "arm_mazzolini/target_spawner.hpp"

TargetSpawner::TargetSpawner() : Node("target_spawner_node")
{
    target_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/target_position", 10);
    laser_sub = this->create_subscription<std_msgs::msg::Bool>(
        "/laser_command", 10,
        std::bind(&TargetSpawner::laser_Callback, this, std_placeholders::_1)
    );
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    timer = this->create_wall_timer(std::chrono::seconds(spawn_period), std::bind(&TargetSpawner::timer_callback, this));
}