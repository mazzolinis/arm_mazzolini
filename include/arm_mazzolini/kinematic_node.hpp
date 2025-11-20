#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <Eigen/Dense>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <cmath>
#include <vector>
#include <string>

namespace arm_mazzolini
{
class WeederNode : public rclcpp::Node
{
public:
    WeederNode();
    
private:
    // link lengths
    double l1;
    double l2;

    // callback period
    const int callback_period_ms = 100;

    enum class TargetStatus {
        NO_TARGET,
        HAS_TARGET,
        ARM_MOVING,
        LASERING,
    };
    TargetStatus target_status;

    // target and robot pose
    Eigen::Vector3d target_position;
    Eigen::Isometry3d relative_pose; // from wheeled robot center to manipulator first joint
    Eigen::Isometry3d new_arm_pose;
    Eigen::Isometry3d arm_pose; // pose of first joint relative to map frame

    const double pose_threshold = 1e-3;   
    
    std::vector<std::string> joint_names = {"joint1", "joint2"};
    void declare_parameters();
    
    // Messages
    control_msgs::action::FollowJointTrajectory::Goal goal_msg;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions goal_options;

    // Subscriptions and publishers
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr joints_client;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr laser_pub;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    rclcpp::TimerBase::SharedPtr timer;
    
    // Callbacks
    void timer_callback();
    void target_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
    void result_callback(const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result);
    
    // Functions
    std::vector<double> compute_ik(const Eigen::Vector3d& position);
    double normalize_angle(double angle);
    void send_joint_trajectory(const std::vector<double>& joint_angles);
};

} // namespace arm_mazzolini
