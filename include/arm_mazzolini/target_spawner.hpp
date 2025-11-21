#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Dense>
#include <random>

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
        void laser_callback(const std_msgs::msg::Bool::SharedPtr msg);

        // Variables
        const int spawn_period = 5; // seconds between laser and new target
        const double min_distance = 0.2;
        const double max_distance = 8.0;
        const double angle_range = M_PI / 3.0;
        
        // Random numbers generators
        std::mt19937 gen;
        std::uniform_real_distribution<double> dist_rand;
        std::uniform_real_distribution<double> angle_rand;
        
}