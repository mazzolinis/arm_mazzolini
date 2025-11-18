#include "arm_mazzolini/node.hpp"

using namespace std::chrono_literals;

namespace arm_mazzolini
{

WeederNode::WeederNode(const rclcpp::NodeOptions & options) : Node("weeder_robot_node", options)
{
  publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);

  timer_ = this->create_wall_timer(std::chrono::seconds(4), std::bind(&WeederNode::publish_trajectory, this));

  RCLCPP_INFO(this->get_logger(), "Weeder node started!");
}

void WeederNode::publish_trajectory()
{
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = {"joint1", "joint2"};

    trajectory_msgs::msg::JointTrajectoryPoint p1, p2, p3, p4, p5;
    p1.positions = {0.0, 0.0};
    p1.time_from_start = rclcpp::Duration(0, 0);

    p2.positions = {0.8, 0.5};
    p2.time_from_start = rclcpp::Duration::from_seconds(1.0);

    p3.positions = {0.0, 0.0};
    p3.time_from_start = rclcpp::Duration::from_seconds(2.0);

    p4.positions = {-0.8, -0.5};
    p4.time_from_start = rclcpp::Duration::from_seconds(3.0);

    p5.positions = {0.0, 0.0};
    p5.time_from_start = rclcpp::Duration::from_seconds(4.0);

    traj.points = {p1, p2, p3, p4, p5};

    publisher_->publish(traj);
    RCLCPP_INFO(this->get_logger(), "Published trajectory command!");
}
}  // namespace arm_mazzolini

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arm_mazzolini::WeederNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
