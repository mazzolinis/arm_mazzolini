#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "pi3hat_moteus_int_msgs/msg/joints_states.hpp"

class JointStateBridge : public rclcpp::Node
{
public:
  JointStateBridge()
  : Node("joint_state_bridge")
  {
    // Parametri configurabili
    this->declare_parameter<std::string>("input_topic", "/omni_controller/joints_state");
    this->declare_parameter<std::string>("output_topic", "/joint_states");
    this->declare_parameter<bool>("use_input_stamp", false);

    const auto input_topic  = this->get_parameter("input_topic").as_string();
    const auto output_topic = this->get_parameter("output_topic").as_string();
    use_input_stamp_        = this->get_parameter("use_input_stamp").as_bool();

    // QoS BEST_EFFORT per compatibilità con il publisher del controller
    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(10))
      .reliability(rclcpp::ReliabilityPolicy::BestEffort);

    // RELIABLE per robot_state_publisher (default)
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10));

    sub_ = this->create_subscription<pi3hat_moteus_int_msgs::msg::JointsStates>(
      input_topic, sub_qos,
      std::bind(&JointStateBridge::callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::JointState>(output_topic, pub_qos);

    RCLCPP_INFO(this->get_logger(),
      "Joint state bridge: '%s' -> '%s'",
      input_topic.c_str(), output_topic.c_str());
  }

private:
  void callback(const pi3hat_moteus_int_msgs::msg::JointsStates::SharedPtr msg)
  {
    sensor_msgs::msg::JointState out;

    // Header: o riusa quello del messaggio in ingresso, o stampa attuale
    out.header = msg->header;
    if (!use_input_stamp_) {
      out.header.stamp = this->now();
    }

    // Copia diretta dei campi standard
    out.name     = msg->name;
    out.position = msg->position;
    out.velocity = msg->velocity;
    out.effort   = msg->effort;

    // Sanity check: tutti i vettori devono avere la stessa size di 'name'
    const size_t n = out.name.size();
    if (!out.position.empty() && out.position.size() != n) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "position size (%zu) != name size (%zu)", out.position.size(), n);
    }
    if (!out.velocity.empty() && out.velocity.size() != n) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "velocity size (%zu) != name size (%zu)", out.velocity.size(), n);
    }
    if (!out.effort.empty() && out.effort.size() != n) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "effort size (%zu) != name size (%zu)", out.effort.size(), n);
    }

    pub_->publish(out);
  }

  rclcpp::Subscription<pi3hat_moteus_int_msgs::msg::JointsStates>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  bool use_input_stamp_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateBridge>());
  rclcpp::shutdown();
  return 0;
}