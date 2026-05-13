#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

class NucHeartbeatNode : public rclcpp::Node
{
public:
  NucHeartbeatNode()
  : Node("nuc_heartbeat")
  {
    this->declare_parameter<double>("rate", 10.0);
    double rate = this->get_parameter("rate").as_double();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliability(rclcpp::ReliabilityPolicy::BestEffort);

    pub_ = this->create_publisher<std_msgs::msg::Empty>("/nuc_heartbeat", qos);

    auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = this->create_wall_timer(period, std::bind(&NucHeartbeatNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
      "NUC heartbeat publishing at %.1f Hz on /nuc_heartbeat", rate);
  }

private:
  void timer_callback()
  {
    pub_->publish(std_msgs::msg::Empty());
  }

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NucHeartbeatNode>());
  rclcpp::shutdown();
  return 0;
}