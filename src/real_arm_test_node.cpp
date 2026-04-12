#include "arm_mazzolini/real_arm_test_node.hpp"

TestRealArmNode::TestRealArmNode() : Node("real_arm_test_node")
{
    command_publisher_ = this->create_publisher<pi3hat_moteus_int_msgs::msg::JointsCommand>("joint_controller/command", 10);

    start_time_ = this->get_clock()->now();
    int period_ms = static_cast<int>(1000.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&TestRealArmNode::timer_callback, this)
    );

    states_subscription_ = this->create_subscription<pi3hat_moteus_int_msgs::msg::JointsStates>(
        "state_broadcaster/joints_state",
        10,
        std::bind(&TestRealArmNode::states_callback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(this->get_logger(), "Real Arm Test Node has started.");
}

void TestRealArmNode::timer_callback()
{
    auto current_time = this->get_clock()->now();
    double t = (current_time - start_time_).seconds();

    pi3hat_moteus_int_msgs::msg::JointsCommand command_msg;
    for (const auto& joint : joints_params) {
        double omega = 2 * M_PI * joint.frequency_hz;
        double position_rad = joint.amplitude_deg * DEG2RAD * std::sin(omega * t);
        // double velocity_rad_s = joint.amplitude_deg * DEG2RAD * omega * std::cos(omega * t);
        double velocity_rad_s = 0.0;

        command_msg.name.push_back(joint.name);
        command_msg.position.push_back(position_rad);
        command_msg.velocity.push_back(velocity_rad_s);
        command_msg.effort.push_back(0.0); // è corretto?
        command_msg.kp_scale.push_back(joint.kp_scale);
        command_msg.kd_scale.push_back(joint.kd_scale);
    }

    command_publisher_->publish(command_msg);
}

void TestRealArmNode::states_callback(const pi3hat_moteus_int_msgs::msg::JointsStates::SharedPtr msg)
{
    for(size_t i = 0; i < msg->name.size(); i++) {
        if(std::isfinite(msg->position[i])) {
            // Check if the joint is one of the joints we are interested in
            size_t idx = std::find(joints_names.begin(), joints_names.end(), msg->name[i]) - joints_names.begin();
            if (idx < joints_names.size()) {
                // Update data
                joints_positions[idx] = msg->position[i];
            }
        }
        else if(std::isfinite(msg->sec_enc_pos[i])) {
            // Check if the joint is one of the joints we are interested in
            size_t idx = std::find(joints_names.begin(), joints_names.end(), msg->name[i]) - joints_names.begin();
            if (idx < joints_names.size()) {
                // Update data
                joints_positions[idx] = msg->sec_enc_pos[i];
            }
        }
        else {
            continue;
        }

        // Same for effort
        if(std::isfinite(msg->effort[i])) {
            // Check if the joint is one of the joints we are interested in
            size_t idx = std::find(joints_names.begin(), joints_names.end(), msg->name[i]) - joints_names.begin();
            if (idx < joints_names.size()) {
                // Update data
                joints_efforts[idx] = msg->effort[i];
            }
        }
    }

    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *(this->get_clock()), 800, "Current joint states: " << std::endl <<
        "Joint 1: position = " << joints_positions[0] << " rad, effort = " << joints_efforts[0] << std::endl <<
        "Joint 2: position = " << joints_positions[1] << " rad, effort = " << joints_efforts[1]);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestRealArmNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}