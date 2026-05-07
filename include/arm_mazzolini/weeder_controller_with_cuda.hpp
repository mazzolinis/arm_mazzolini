#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
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
#include <thread>
#include <atomic>

#include "arm_mazzolini/sphere_detector.hpp"
#include "arm_mazzolini/arm_kinematic.hpp"
#include "pi3hat_moteus_int_msgs/msg/joints_command.hpp"
#include "pi3hat_moteus_int_msgs/msg/joints_states.hpp"

namespace arm_mazzolini
{
    class WeederController : public rclcpp::Node
    {
        public:
            WeederController();
            ~WeederController();

        private:

        // Parameters
        bool use_sim_time;
        bool use_real_hw;

        // Parameters from yaml
        double l1;
        double l2;
        std::string camera_rgb_topic;
        std::string camera_depth_topic;
        std::string camera_info_topic;
        bool filtering;

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
        bool camera_initialized = false;
        std::vector<Eigen::Vector3d> target_buffer;
        size_t image_buffer_size;
        int frames_delay;
        int current_frame_index = 0;
        int roi_size;
        int morph_kernel_size;
        int depth_roi_size;
        MaskType mask_type;
    
        // Mobile robot pose (and threshold)
        Eigen::Isometry3d old_pose; // pose of mobile robot base relative to map frame
        Eigen::Isometry3d new_pose; // same but at next step
        const double pose_threshold = 1e-3;   
        
        // Joint parameters
        std::vector<std::string> joint_names = {"joint1", "joint2"};
        std::vector<double> kp_scale = {1.0, 1.0};
        std::vector<double> kd_scale = {1.0, 1.0};
        const std::vector<double> initial_joint_values = {-M_PI/3, M_PI/3}; 
        std::vector<double> joint_states = initial_joint_values;
        std::vector<double> last_joint_angles;
        const double joint_tolerance = 2e-4; // radians
    

        // Timers
        rclcpp::TimerBase::SharedPtr pose_timer;
        rclcpp::TimerBase::SharedPtr control_timer;
        int control_dt;
        int trajectory_dt;

        // Thread
        std::thread keyboard_thread;
        std::atomic<bool> keyboard_thread_active_{false};

        // Control and filter gains
        Eigen::Vector3d target_feature = Eigen::Vector3d::Zero(); // u, v, Z feature of the target in camera frame
        Eigen::Vector3d target_position = Eigen::Vector3d::Zero();  // x, y, z position of the target in kinematic_link frame
        Eigen::Vector3d target_camera_position = Eigen::Vector3d::Zero();  // x, y, z position of the target in camera_kinematic frame
        Eigen::Vector3d desired_feature; // u, v, Z desired feature of the target in camera frame
        Eigen::Vector3d desired_position; // x, y, z desired position of the target in camera_kinematic frame, usually [0, 0, Z]
        Eigen::Vector3d e_s_filtered = Eigen::Vector3d::Zero();
        Eigen::Vector3d e_p_filtered = Eigen::Vector3d::Zero();
        // Eigen::Vector3d alpha; // Not used right now
        Eigen::Vector3d beta;
        Eigen::DiagonalMatrix<double, 3> lambda_IBVS;
        Eigen::DiagonalMatrix<double, 3> lambda_PBVS;
        double control_threshold; 

        // Messages
        control_msgs::action::FollowJointTrajectory::Goal goal_msg;
        rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions goal_options;

        // Subscriptions and publishers
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
        rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr joints_client;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr laser_pub;
        rclcpp::Publisher<pi3hat_moteus_int_msgs::msg::JointsCommand>::SharedPtr command_publisher_;
        rclcpp::Subscription<pi3hat_moteus_int_msgs::msg::JointsStates>::SharedPtr states_subscription_;

        // Image QoS subscriber
        message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
        message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,sensor_msgs::msg::Image,sensor_msgs::msg::CameraInfo> SyncPolicy;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

        // TF2 parameters
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        rclcpp::Time last_warning_time;
        int trajectory_time_ms;
        const uint64_t message_throttle_ms = 1000; // for INFO and WARN messages using THROTTLE

        // Functions
        void declare_and_get_parameters();   
        void send_joint_trajectory(const std::vector<double>& joint_angles);
        bool vectors_are_equal(const std::vector<double>& vec1, const std::vector<double>& vec2);
        void activate_laser();
        std::vector<double> IBVS_control(Eigen::Vector3d& feature);
        std::vector<double> PBVS_control(Eigen::Vector3d& center);
        
        // Callbacks
        void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
        void pose_timer_callback();
        void pose_callback(const geometry_msgs::msg::TransformStamped msg);
        void result_callback(const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result);
        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr rgb_msg,
                            const sensor_msgs::msg::Image::ConstSharedPtr depth_msg,
                            const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg);
        void control_callback();
        void real_states_callback(const pi3hat_moteus_int_msgs::msg::JointsStates::SharedPtr msg);

        // Other packages
        std::unique_ptr<SphereDetector> sphere_detector;
        std::unique_ptr<ArmKinematic> arm_kinematic;
   
    };

} // namespace arm_mazzolini

