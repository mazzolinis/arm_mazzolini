#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/empty.hpp>

class RvizTargetVisualizer : public rclcpp::Node
{
public:
    RvizTargetVisualizer();

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_sub_;
    
    void target_callback(const geometry_msgs::msg::Point::SharedPtr msg);
    void clear_callback(const std_msgs::msg::Empty::SharedPtr msg);
};
