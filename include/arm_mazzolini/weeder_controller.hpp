#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "message_filters/synchronizer.hpp"
#include "rmw/qos_profiles.h"

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <memory>

#include "arm_mazzolini/sphere_detector.hpp"
#include "arm_mazzolini/arm_kinematic.hpp"

namespace arm_mazzolini
{
    class WeederController : public rclcpp::Node
    {
        public:
            WeederController();

        private:

        // =====================================================================
        //                        TODO: RIORDINARE LE VARIABILI!!!!!
        // =====================================================================        

        // Parameters from yaml
        double l1;
        double l2;
        std::array<double, 3> translation;
        std::array<double, 3> rotation;
        std::string camera_rgb_topic;
        std::string camera_depth_topic;
        std::string camera_info_topic;

        // callback period
        int tf_callback_period_ms;

        enum class ControllerStatus {
        NO_TARGET,
        HAS_TARGET,
        ARM_MOVING,
        POSITIONING,
        LASERING,
        };

        ControllerStatus controller_status;

        // Image parameters
        Eigen::Vector3d target_position;
        // Eigen::Vector3d target_relative_position; // relative to the camera
        std::vector<Eigen::Vector3d> target_buffer;
        size_t image_buffer_size;
        int frames_delay;
        int current_frame_index = 0;
    
        Eigen::Isometry3d old_pose; // pose of mobile robot base relative to map frame
        Eigen::Isometry3d new_pose; // same but at next step

        const double pose_threshold = 1e-3;   
        
        std::vector<std::string> joint_names = {"joint1", "joint2"};
        void declare_and_get_parameters();    

        // Messages
        control_msgs::action::FollowJointTrajectory::Goal goal_msg;
        rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions goal_options;

        // Subscriptions and publishers
        rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr joints_client;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr laser_pub;

        // Image QoS subscriber
        message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
        message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,sensor_msgs::msg::Image,sensor_msgs::msg::CameraInfo> SyncPolicy;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

        // TF2 parameters
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Time last_warning_time;
        double warning_period = 2.0; // seconds
        int trajectory_time_ms = 50;
        
        // Callbacks
        void timer_callback();
        void pose_callback(const geometry_msgs::msg::TransformStamped msg);
        void result_callback(const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result);
        void image_callback(const sensor_msgs::msg::Image::SharedPtr rgb_msg,
                            const sensor_msgs::msg::Image::SharedPtr depth_msg,
                            const sensor_msgs::msg::CameraInfo::SharedPtr info_msg);
        void send_joint_trajectory(const std::vector<double>& joint_angles);

        // Other packages
        std::unique_ptr<SphereDetector> sphere_detector;
        std::unique_ptr<ArmKinematic> arm_kinematic;
   
    };

} // namespace arm_mazzolini

