#pragma once
#include <rclcpp/rclcpp.hpp>
// #include <node_options.hpp>
// #include <chrono>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace arm_mazzolini
{
class WeederNode : public rclcpp::Node
{
public:
  WeederNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void publish_trajectory();

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace arm_mazzolini
