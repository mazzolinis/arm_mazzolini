#pragma once
#include <rclcpp/rclcpp.hpp>
#include <pi3hat_moteus_int_msgs/msg/joints_command.hpp>
#include <pi3hat_moteus_int_msgs/msg/joints_states.hpp>
#include <cmath>
#include <string>
#include <vector>
#include <stdexcept>

class TestRealArmNode : public rclcpp::Node
{
public:
    TestRealArmNode();


private:
    static constexpr double DEG2RAD = M_PI / 180.0;
    struct JointParams {
        std::string name;
        double amplitude_deg;
        double frequency_hz;
        double kp_scale;
        double kd_scale;
    };

    JointParams joint1_params = {"joint1", 20.0, 0.25, 0.5, 0.5};
    JointParams joint2_params = {"joint2", 10.0, 0.45, 0.5, 0.5};  
    std::vector<std::string> joints_names = {joint1_params.name, joint2_params.name};
    std::vector<double> joints_positions = {0.0, 0.0};
    std::vector<double> joints_efforts = {0.0, 0.0};
    std::vector<JointParams> joints_params = {joint1_params, joint2_params};
    rclcpp::Publisher<pi3hat_moteus_int_msgs::msg::JointsCommand>::SharedPtr command_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    double publish_rate_hz_ = 20.0;

    rclcpp::Subscription<pi3hat_moteus_int_msgs::msg::JointsStates>::SharedPtr states_subscription_;

    void timer_callback();
    void states_callback(const pi3hat_moteus_int_msgs::msg::JointsStates::SharedPtr msg);
    // void initialize_joint_params();

};