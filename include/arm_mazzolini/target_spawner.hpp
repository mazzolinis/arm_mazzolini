#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Dense>
#include <random>
#include <fstream>
#include <mutex>

class TargetSpawner : public rclcpp::Node
{
    public:
        TargetSpawner();
        ~TargetSpawner();

    private:

        double world_height;
        Eigen::Vector3d target_position;
        Eigen::Isometry3d robot_pose_;

        // Logging
        std::string out_file;
        std::mutex buffer_mutex;
        // std::vector<std::tuple<double, double, double>> buffer;
        std::vector<std::tuple<double, double, double, double>> buffer;
        
        // Publisher
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr clear_pub;

        // Subscriber
        // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr laser_sub;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr laser_position_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr ground_truth_sub;

        // Timer and buffer
        rclcpp::TimerBase::SharedPtr timer;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        std::shared_ptr<rclcpp::Subscription<tf2_msgs::msg::TFMessage>> gz_sub_;

        // Callbacks
        void timer_callback();
        void laser_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
        // void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
        void ground_truth_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);

        // Variables
        const int spawn_period = 2; // seconds between laser and new target
        const double min_distance = 0.1;
        const double max_distance = 0.5;
        const double angle_range = M_PI / 6;
        // adding a new area to spawn target
        const double lato = 0.2;
        // Eigen::Vector3d center = Eigen::Vector3d(-0.08, -0.78, 1.27); 
        Eigen::Vector3d center = Eigen::Vector3d(0.0, 1.3, 0.33); 
        Eigen::Isometry3d world_to_base_link;
        Eigen::Vector3d target_position_world;
        
        // Random numbers generators
        std::mt19937 gen;
        std::uniform_real_distribution<double> dist_rand;
        std::uniform_real_distribution<double> angle_rand;
        std::uniform_real_distribution<double> lato_rand;
        
};