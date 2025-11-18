#include "arm_mazzolini/target_visualizer_rviz.hpp"

RvizTargetVisualizer::RvizTargetVisualizer() 
: Node("rviz_target_visualizer")
{
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/target_marker", 10);
    
    target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/target_position", 10,
        [this](const geometry_msgs::msg::Point::SharedPtr msg) {
            this->target_callback(msg);
        });
    
    clear_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/clear_target", 10,
        [this](const std_msgs::msg::Empty::SharedPtr msg) {
            this->clear_callback(msg);
        });
}

void RvizTargetVisualizer::target_callback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->now();
    marker.ns = "target";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position = *msg;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    
    marker_pub_->publish(marker);
}

void RvizTargetVisualizer::clear_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->now();
    marker.ns = "target";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    
    marker_pub_->publish(marker);
}