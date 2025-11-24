#include "arm_mazzolini/target_spawner.hpp"

TargetSpawner::TargetSpawner() : Node("target_spawner_node"),
    gen(std::random_device{}()),
    dist_rand(min_distance, max_distance),
    angle_rand(-angle_range/2, angle_range/2)
{
    target_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/target_position", 10);
    laser_sub = this->create_subscription<std_msgs::msg::Bool>(
        "/laser_command", 10,
        std::bind(&TargetSpawner::laser_callback, this, std::placeholders::_1)
    );
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    timer = rclcpp::create_timer(
        this,
        this->get_clock(),
        rclcpp::Duration(std::chrono::seconds(spawn_period)), 
        std::bind(&TargetSpawner::timer_callback, this)
    );
    // Following function only if the previous one doesn't work
    // timer = rclcpp::create_timer(
    //     this->get_node_base_interface(),
    //     this->get_node_timers_interface(),
    //     this->get_clock(),
    //     rclcpp::Duration(std::chrono::seconds(spawn_period)),
    //     std::bind(&TargetSpawner::timer_callback, this),
    //     this->get_default_callback_group()
    // );
}

void TargetSpawner::laser_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data == true) {
        RCLCPP_INFO(this->get_logger(), "Laser worked fine, new target arriving...");
        timer->reset();
    }
    if (msg->data == false) {
        RCLCPP_WARN(this->get_logger(), "Laser failed, what now?");
        timer->reset();
    }
}

void TargetSpawner::timer_callback()
{
    // Get robot pose
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
        transformStamped = tf_buffer->lookupTransform("odom", "base_link", tf2::TimePointZero);
    }
    // TODO: change link names into variables, hard coded names are not good
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform base_link to odom: %s", ex.what());
        return;
    }

    Eigen::Isometry3d robot_pose = tf2::transformToEigen(transformStamped);

    // Generate random target in front of robot
    double distance = dist_rand(gen);
    double angle = angle_rand(gen);

    Eigen::Vector3d offset(distance * std::cos(angle), distance * std::sin(angle), 0.0);
    Eigen::Vector3d target_position = robot_pose.translation() + robot_pose.rotation() * offset;
    target_position.z() = 0.0; // ground level

    // Publish target
    geometry_msgs::msg::PointStamped target_msg;
    target_msg.header.stamp = this->now();
    target_msg.header.frame_id = "odom";
    target_msg.point.x = target_position.x();
    target_msg.point.y = target_position.y();
    target_msg.point.z = target_position.z();

    target_pub->publish(target_msg);

    timer->cancel();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetSpawner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}