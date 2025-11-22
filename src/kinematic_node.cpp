#include "arm_mazzolini/kinematic_node.hpp"

using namespace arm_mazzolini;

WeederNode::WeederNode() : Node("weeder_node")
{
    // Parameters
    declare_parameters();
    l1 = this->get_parameter("link1_length").as_double();
    l2 = this->get_parameter("link2_length").as_double();
    const auto translation = this->get_parameter("base_frame_transform.translation").as_double_array();
    const auto rotation = this->get_parameter("base_frame_transform.rotation").as_double_array();

    // pose initialization
    Eigen::Vector3d t(translation[0], translation[1], translation[2]);
    Eigen::Matrix3d R = 
        Eigen::AngleAxisd(rotation[0] *M_PI/180.0, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(rotation[1] *M_PI/180.0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(rotation[2] *M_PI/180.0, Eigen::Vector3d::UnitX());
    
    relative_pose.linear() = R;
    relative_pose.translation() = t;
    arm_pose = relative_pose; // need to initialize pose
    
    // Subscribers
    target_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/target_position", 10,
        std::bind(&WeederNode::target_callback, this, std::placeholders::_1)
    );

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    timer = this->create_timer(std::chrono::milliseconds(callback_period_ms), std::bind(&WeederNode::timer_callback, this));

    // Publishers and Action Clients
    joints_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        this,
        "/joint_trajectory_controller/follow_joint_trajectory"
    );
    if (action_client->wait_for_action_server(std::chrono::seconds(10)) == false) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        RCLCPP_INFO(this->get_logger(), "Shutting down...");
        rclcpp::shutdown();
    }
    goal_options.result_callback = std::bind(&WeederNode::result_callback, this, std::placeholders::_1);
    goal_msg.trajectory.joint_names = joint_names;

    laser_pub = this->create_publisher<std_msgs::msg::Bool>("/laser_command", 10);

    // Initial status
    target_status = TargetStatus::NO_TARGET;
}

void WeederNode::declare_parameters()
{
    try {
        this->declare_parameter("link1_length", double());
        this->declare_parameter("link2_length", double());  
        this->declare_parameter("base_frame_transform.translation", std::vector<double>());
        this->declare_parameter("base_frame_transform.rotation", std::vector<double>());
    } 
    catch(const rclcpp::exceptions::InvalidParameterTypeException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Error in params declaration: %s", ex.what());
        RCLCPP_INFO(this->get_logger(), "Shutting down...");
        rclcpp::shutdown();
    }
}

void WeederNode::timer_callback()
{
    try {
        geometry_msgs::msg::TransformStamped pose_message = 
        tf_buffer->lookupTransform("odom", "base_link", tf2::TimePointZero);

        pose_callback(&pose_message);
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(); "Could not transform odom to base_link: %s", ex.what());
        return;
    }
}

void WeederNode::pose_callback(const geometry_msgs::msg::TranformStamped::SharedPtr msg)
{
    // ------------------- TODO: What do I do of header? ----------------------

    Eigen::Isometry3d wheeled_robot_pose = tf2::transformToEigen(&msg); // convertion out of switch case because i could change message type

    switch (target_status) {
        case TargetStatus::NO_TARGET:
            // TODO: maybe do something if I don't have a target?
            break;

        case TargetStatus::HAS_TARGET:
            new_pose = wheeled_robot_pose * relative_pose;
            Eigen::Vector3d relative_position = new_pose.inverse() * target_position;
            double r = std::sqrt(relative_position.x()*relative_position.x() + relative_position.y()*relative_position.y());

            if (r > (l1+l2)) {
                RCLCPP_INFO(this->get_logger(), "Get closer, target out of reach.");
                return;
            }
            else if (relative_position.x() < 0) { // TODO: define exclusion zones
                RCLCPP_INFO(this->get_logger(), "Target behind the robot, cannot reach.");
                return;
            }
            else {
                if (new_pose.isApprox(arm_pose, pose_threshold)) {
                    std::vector<double> joint_angles = compute_ik(relative_position);
                    send_joint_trajectory(joint_angles);
                    arm_pose = new_pose;
                }
                else {
                    RCLCPP_INFO(this->get_logger(), "STOP!!!");
                    arm_pose = new_pose;
                }
            }
            break;
            
        case TargetStatus::ARM_MOVING:
            new_pose = wheeled_robot_pose * relative_pose;
            if (!new_pose.isApprox(arm_pose, pose_threshold)) {
                RCLCPP_INFO(this->get_logger(), "STOP!!!");
                joint_client->async_cancel_all_goals();
                target_status = TargetStatus::HAS_TARGET;
                arm_pose = new_pose;
            }
            break;

        case TargetStatus::LASERING:
            break; // TODO: what happens here? It's critical
    }

    // TODO: check callback completeness
}


std::vector<double> WeederNode::compute_ik(const Eigen::Vector3d& position)
{
    double x = position.x();
    double y = position.y();
    double r = std::sqrt(x*x + y*y);

    // Check for errors
    if (r > (l1+l2) || r < std::abs(l1-l2)) {
        RCLCPP_ERROR(this->get_logger(), "Target position is out of reach, check previous computations.");
        return {};
    }
    else {
        double cos_theta2 = (r*r - l1*l1 - l2*l2) / (2 * l1 * l2);

        // Numerical safety for acos
        if (cos_theta2 > 1.0) cos_theta2 = 1.0;
        else if (cos_theta2 < -1.0) cos_theta2 = -1.0;

        double sin_theta2 = std::sqrt(1 - cos_theta2 * cos_theta2); // elbow_down solution
        double theta2 = std::atan2(sin_theta2, cos_theta2);

        double k1 = l1 + l2 * cos_theta2;
        double k2 = l2 * sin_theta2;
        double theta1 = std::atan2(y, x) - std::atan2(k2, k1);

        // Normalize angles
        theta1 = normalize_angle(theta1);
        theta2 = normalize_angle(theta2);

        return {theta1, theta2};
    }
}

double WeederNode::normalize_angle(double angle)
{
    while (angle < -M_PI) angle += 2 * M_PI;
    while (angle > M_PI) angle -= 2 * M_PI;
    return angle;
}

void WeederNode::target_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    target_position = tf2::fromMsg(msg->point);
    target_status = TargetStatus::HAS_TARGET;
}

void WeederNode::send_joint_trajectory(const std::vector<double>& joint_angles)
{
    goal_msg.trajectory.points.clear();
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = joint_angles;
    point.time_from_start = rclcpp::Duration(std::chrono::milliseconds(2));
    goal_msg.trajectory.points.push_back(point);

    joints_client->async_send_goal(goal_msg, goal_options);
    target_status = TargetStatus::ARM_MOVING;
}

void WeederNode::result_callback(const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "BYE BYE PLANT!");
            target_status = TargetStatus::LASERING;
            laser_pub->publish(true); // TODO: check what to do in LASERING status
            target_status = TargetStatus::NO_TARGET;
            break;

            // TODO: handle other cases
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Joint trajectory execution aborted.");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Joint trajectory execution canceled.");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
            return;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WeederNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}