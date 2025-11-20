#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class TargetSpawner : public rclcpp::Node
{
    public:
        TargetSpawner();

    private:
        // Publisher
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub;

        // Subscriber
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr laser_sub;

        // Timer and buffer
        rclcpp::TimerBase::SharedPtr timer;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;

        // Callbacks
        void timer_callback();
        void laser_Callback(const std_msgs::msg::Bool::SharedPtr msg);
        void pose_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
}