#include "arm_mazzolini/target_spawner.hpp"

TargetSpawner::TargetSpawner() : Node("target_spawner_node"),
    gen(std::random_device{}()),
    dist_rand(min_distance, max_distance),
    angle_rand(-angle_range/2, angle_range/2),
    lato_rand(-lato/2, lato/2)
{
    try{
        this->declare_parameter("world_height", double());
        world_height = this->get_parameter("world_height").as_double();
        this->declare_parameter("output_file", "/home/simone/scrivania/target_data.csv");
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
    pose_sub = create_subscription<geometry_msgs::msg::PoseArray>(
      "/model/robot/pose", 10,
      std::bind(&TargetSpawner::pose_callback, this, std::placeholders::_1)
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

void TargetSpawner::pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    if (msg->poses.empty()) return;

    geometry_msgs::msg::TransformStamped tf;
    tf.header = msg->header;
    tf.transform.translation.x = msg->poses[0].position.x;
    tf.transform.translation.y = msg->poses[0].position.y;
    tf.transform.translation.z = msg->poses[0].position.z;
    tf.transform.rotation = msg->poses[0].orientation;

    robot_pose_ = tf2::transformToEigen(tf);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received robot pose: [%.2f, %.2f, %.2f]", robot_pose_.translation().x(), robot_pose_.translation().y(), robot_pose_.translation().z());

}

void TargetSpawner::timer_callback()
{
    // Get robot pose
    geometry_msgs::msg::TransformStamped transformStamped;

    // RCLCPP_INFO(this->get_logger(), "============================================================================");
    // RCLCPP_INFO(this->get_logger(), "Spawning new target...");
    // RCLCPP_INFO(this->get_logger(), "============================================================================");

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
    target_msg.header.frame_id = "map";
    target_msg.point.x = target_position.x();
    target_msg.point.y = target_position.y();
    target_msg.point.z = target_position.z();

    target_pub->publish(target_msg);

    timer->cancel();
}

void TargetSpawner::laser_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    // Eigen::Vector3d lasered_position, laser_relative_position;
    // tf2::fromMsg(msg->point, laser_relative_position);

    
    // lasered_position = robot_pose_ * laser_relative_position;
    

    // // RCLCPP_INFO(this->get_logger(), "LASERED POSITION: [%.2f, %.2f, %.2f]", lasered_position.x(), lasered_position.y(), lasered_position.z());
    // // RCLCPP_INFO(this->get_logger(), "REAL POSITION: [%.2f, %.2f, %.2f]", target_position.x(), target_position.y(), target_position.z());
    // // Save instead of printing data
    // double x = target_position.x() - lasered_position.x();
    // double y = target_position.y() - lasered_position.y();
    // double r = std::sqrt(x*x + y*y);
    // std::tuple<double, double, double> data_entry(x, y, r);
    // std::lock_guard<std::mutex> guard(buffer_mutex);
    // buffer.push_back(data_entry);

    Eigen::Vector3d lasered_position;
    tf2::fromMsg(msg->point, lasered_position);

    double error = std::sqrt(std::pow(target_position.x() - lasered_position.x(), 2) + std::pow(target_position.y() - lasered_position.y(), 2));

    // Try printing more data to see the error
    std::tuple<double, double, double, double, double> data_entry(
        target_position.x(), target_position.y(), 
        lasered_position.x(), lasered_position.y(),
        error
    );
    std::lock_guard<std::mutex> guard(buffer_mutex);
    buffer.push_back(data_entry);

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
    file << "target_x,target_y,lasered_x,lasered_y,error" << std::endl;
    for (const auto& entry : buffer) {
        file << std::get<0>(entry) << ","
             << std::get<1>(entry) << ","
             << std::get<2>(entry) << ","
             << std::get<3>(entry) << ","
             << std::get<4>(entry) << std::endl;
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