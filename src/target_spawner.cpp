#include "arm_mazzolini/target_spawner.hpp"

TargetSpawner::TargetSpawner() : Node("target_spawner_node"),
    gen(std::random_device{}()),
    dist_rand(min_distance, max_distance),
    angle_rand(-angle_range/2, angle_range/2)
{
    try{
        this->declare_parameter("world_height", double());
        world_height = this->get_parameter("world_height").as_double();
    }
    catch (const rclcpp::ParameterTypeException &e) {
        RCLCPP_ERROR(this->get_logger(), "Parameter 'world_height' not found or of wrong type, using default 2.0 m");
        world_height = 2.0;
    }

    // Publishers
    target_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/target_position", 10);
    // laser_sub = this->create_subscription<std_msgs::msg::Bool>(
    //     "/laser_command", 10,
    //     std::bind(&TargetSpawner::laser_callback, this, std::placeholders::_1)
    // );
    clear_pub = this->create_publisher<std_msgs::msg::Bool>("/clear_target",10);

    // Subscriber
    laser_position_sub = this->create_subscription<geometry_msgs::msg::Point>(
        "/lasered_position",10, std::bind(&TargetSpawner::laser_callback, this, std::placeholders::_1)
    );

    // TF
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    timer = rclcpp::create_timer(
        this,
        this->get_clock(),
        rclcpp::Duration(std::chrono::seconds(spawn_period)), 
        std::bind(&TargetSpawner::timer_callback, this)
    );
}

void TargetSpawner::laser_callback(const geometry_msgs::msg::Point msg)
{
    Eigen::Vector3d lasered_position;
    tf2::fromMsg(msg, lasered_position);

    RCLCPP_INFO(this->get_logger(), "LASERED POSITION: [%.2f, %.2f, %.2f]", lasered_position.x(), lasered_position.y(), lasered_position.z());
    RCLCPP_INFO(this->get_logger(), "REAL POSITION: [%.2f, %.2f, %.2f]", target_position.x(), target_position.y(), target_position.z());


    // TODO: change it to false if error is too large
    std_msgs::msg::Bool bool_msg;
    bool_msg.data = true;
    clear_pub->publish(bool_msg);
}

void TargetSpawner::timer_callback()
{
    // Get robot pose
    geometry_msgs::msg::TransformStamped transformStamped;
    
    // check if transform is available (non-blocking)
    if (!tf_buffer->canTransform("odom", "base_link", tf2::TimePointZero)) {
        RCLCPP_WARN(this->get_logger(), "Transform odom->base_link not available yet");
        timer->reset(); // retry next period
        return;
    }
    
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

    target_position.setZero();
    target_position = robot_pose.translation() + robot_pose.rotation() * offset;
    target_position.z() = world_height; // ground level

    // manual override (TODO: remove)
    target_position.x() = 0.1 + (distance - (max_distance + min_distance)/2) * std::sin(angle) / 8.0;
    target_position.y() = -0.8 + (distance - (max_distance + min_distance)/2) * std::cos(angle) / 8.0;


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