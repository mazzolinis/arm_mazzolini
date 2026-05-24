#include "arm_mazzolini/target_spawner.hpp"

TargetSpawner::TargetSpawner() : Node("target_spawner"),
    gen(std::random_device{}()),
    dist_rand(min_distance, max_distance),
    angle_rand(-angle_range/2, angle_range/2),
    lato_rand(-lato/2, lato/2)
{
    try{
        this->declare_parameter("world_height", double());
        world_height = this->get_parameter("world_height").as_double();
        this->declare_parameter("output_file", "/home/simone/Scrivania/second_test_data.csv");
        out_file = this->get_parameter("output_file").as_string();
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

    // Subscribers
    laser_position_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/lasered_position",10, std::bind(&TargetSpawner::laser_callback, this, std::placeholders::_1)
    );

    // TF
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Timer
    timer = rclcpp::create_timer(
        this,
        this->get_clock(),
        rclcpp::Duration(std::chrono::seconds(spawn_period)), 
        std::bind(&TargetSpawner::timer_callback, this)
    );

    ground_truth_sub = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/world/agricultural_world/pose/info", 10, std::bind(&TargetSpawner::ground_truth_callback, this, std::placeholders::_1)
    );

}

void TargetSpawner::ground_truth_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    for (const auto & tf : msg->transforms) {
        if (tf.child_frame_id == "weeder_robot") {
            // convert to Eigen
            world_to_base_link = tf2::transformToEigen(tf.transform);
            RCLCPP_DEBUG_ONCE(this->get_logger(), "Ground truth received, updated world_to_base_link");
        }
        else if (tf.child_frame_id.rfind("target_sphere", 0) == 0) {
            target_position_world.x() = tf.transform.translation.x;
            target_position_world.y() = tf.transform.translation.y;
            target_position_world.z() = tf.transform.translation.z;
            RCLCPP_DEBUG_ONCE(this->get_logger(), "Ground truth target position received");
        }
    }
}

void TargetSpawner::timer_callback()
{
    // Get robot pose
    geometry_msgs::msg::TransformStamped transformStamped;

    RCLCPP_DEBUG(this->get_logger(), "============================================================================");
    RCLCPP_DEBUG(this->get_logger(), "Spawning new target...");
    RCLCPP_DEBUG(this->get_logger(), "============================================================================");

    // ===========================================================================================================================================
    //              TODO: fare chiarezza qui
    // ===========================================================================================================================================

    // // check if transform is available (non-blocking)
    // if (!tf_buffer->canTransform("odom", "kinematic_link", tf2::TimePointZero)) {
    //     RCLCPP_WARN(this->get_logger(), "Transform odom->kinematic_link not available yet");
    //     timer->reset(); // retry next period
    //     return;
    // }
    
    // try {
    //     transformStamped = tf_buffer->lookupTransform("odom", "kinematic_link", tf2::TimePointZero);
    // }
    // // TODO: change link names into variables, hard coded names are not good
    // catch (tf2::TransformException &ex) {
    //     RCLCPP_WARN(this->get_logger(), "Could not transform kinematic_link to odom: %s", ex.what());
    //     return;
    // }

    // Eigen::Isometry3d robot_pose = tf2::transformToEigen(transformStamped);

    // Generate random target in front of robot
    // double distance = dist_rand(gen);
    // double angle = angle_rand(gen);

    // Eigen::Vector3d offset(distance * std::sin(angle), -distance * std::cos(angle), 0.0);
    // Eigen::Vector3d offset(distance * std::cos(angle), distance * std::sin(angle), 0.0);

    // target_position.setZero();
    // target_position = robot_pose.translation() + robot_pose.rotation() * offset;
    // target_position.z() = world_height; // ground level

    // // manual override (TODO: remove)
    // target_position.x() = (distance - (max_distance + min_distance)/2) * std::sin(angle) / 8.0;
    // target_position.y() = -0.9 + (distance - (max_distance + min_distance)/2) * std::cos(angle) / 8.0;

    // another manual override for testing
    // target_position = Eigen::Vector3d(0.0, -1.0, world_height);

    // Here is another try
    double x = center.x() + lato_rand(gen);
    double y = center.y() + lato_rand(gen);
    target_position = Eigen::Vector3d(x, y, center.z());


    // Publish target
    geometry_msgs::msg::PointStamped target_msg;
    target_msg.header.stamp = this->now();
    target_msg.header.frame_id = "world";
    target_msg.point.x = target_position.x();
    target_msg.point.y = target_position.y();
    target_msg.point.z = target_position.z();

    target_pub->publish(target_msg);

    RCLCPP_INFO(this->get_logger(), "New target spawned at: [%.2f, %.2f, %.2f]", target_position.x(), target_position.y(), target_position.z());

    timer->cancel();
}

void TargetSpawner::laser_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{

    // =======================
    // PARTE NUOVA CON GROUND TRUTH
    // =======================
    Eigen::Vector3d base_to_laser;
    try {
        auto laser_tf = tf_buffer->lookupTransform(
            "base_link", "laser", tf2::TimePointZero, tf2::durationFromSec(0.1));
        base_to_laser.x() = laser_tf.transform.translation.x;
        base_to_laser.y() = laser_tf.transform.translation.y;
        base_to_laser.z() = laser_tf.transform.translation.z;
    }
    catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "base_link->laser non disponibile: %s", ex.what());
        return;
    }

    Eigen::Vector3d laser_position_world = world_to_base_link * base_to_laser;

    std::tuple<double, double, double, double> data_entry(
        target_position_world.x(), target_position_world.y(), 
        laser_position_world.x(), laser_position_world.y()
    );
    std::lock_guard<std::mutex> guard(buffer_mutex);
    buffer.push_back(data_entry);

    // // ===============================
    // // QUESTA PARTE ERA FUNZIONANTE
    // // ===============================
    // Eigen::Vector3d lasered_position;
    // tf2::fromMsg(msg->point, lasered_position);

    // std::tuple<double, double, double, double> data_entry(
    //     target_position.x(), target_position.y(), 
    //     lasered_position.x(), lasered_position.y()
    // );
    // std::lock_guard<std::mutex> guard(buffer_mutex);
    // buffer.push_back(data_entry);

    // ===============================
    //  DO NOT REMOVE THIS PART
    // ===============================
    // TODO: change it to false if error is too large
    std_msgs::msg::Bool bool_msg;
    bool_msg.data = true;
    clear_pub->publish(bool_msg);
    timer->reset();
}

TargetSpawner::~TargetSpawner()
{
    // Write buffered data to file
    std::lock_guard<std::mutex> lock(buffer_mutex);
    std::ofstream file;
    file.open(out_file, std::ios_base::app); // append mode
    if(!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open output file: %s", out_file.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Writing target data to file: %s", out_file.c_str());

    // // Header
    // file << "x,y,error" << std::endl;

    // for (const auto& entry : buffer) {
    //     file << std::get<0>(entry) << "," << std::get<1>(entry) << "," << std::get<2>(entry) << std::endl;
    // }
    // file.close();

    // New format with more data
    file << "target_x,target_y,lasered_x,lasered_y" << std::endl;
    for (const auto& entry : buffer) {
        file << std::get<0>(entry) << ","
             << std::get<1>(entry) << ","
             << std::get<2>(entry) << ","
             << std::get<3>(entry) << std::endl;
    }
    file.close();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetSpawner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}