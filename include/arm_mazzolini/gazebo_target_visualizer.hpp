#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <gazebo_msgs/srv/delete_entity.hpp>
#include <std_msgs/msg/bool.hpp>

class GazeboTargetVisualizer : public rclcpp::Node
{
public:
    GazeboTargetVisualizer();

private:
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client_;
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clear_sub_;
    
    std::string current_target_name_;
    
    void target_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void clear_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void delete_current_target();
};