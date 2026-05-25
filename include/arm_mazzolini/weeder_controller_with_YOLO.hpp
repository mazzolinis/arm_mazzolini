#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/empty.hpp"
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

#include "arm_mazzolini/weed_detector.hpp"
#include "arm_mazzolini/arm_kinematic.hpp"
#include "pi3hat_moteus_int_msgs/msg/joints_command.hpp"
#include "pi3hat_moteus_int_msgs/msg/joints_states.hpp"

namespace arm_mazzolini
{
    class WeederControllerWithYolo : public rclcpp::Node
    {
        public:
            WeederControllerWithYolo();
            ~WeederControllerWithYolo();

        private:

        // Parameters
        bool use_sim_time;

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
        int detection_period_ms;
        rclcpp::Time last_detection_time;
        int current_frame_index = 0;
        float confidence_threshold;
        int depth_roi_size;
        int CUDA_device_id;
    
        // Mobile robot pose (and threshold)
        Eigen::Isometry3d old_pose; // pose of mobile robot base relative to map frame
        Eigen::Isometry3d new_pose; // same but at next step
        const double pose_threshold = 1e-3;   
        
        // Joint parameters
        std::vector<std::string> joint_names = {"joint1", "joint2"};
        std::vector<double> kp_scale;
        std::vector<double> kd_scale;
        const std::vector<double> initial_joint_values = {-M_PI/4, M_PI/4}; 
        std::vector<double> joint_states = initial_joint_values;
        std::vector<double> last_joint_angles;
        bool return_motion_active_ = false;
        double return_motion_elapsed_ = 0.0;
        rclcpp::Time return_motion_start_time_;
        rclcpp::Time return_motion_last_tick_;
        std::vector<double> return_motion_start_positions_;
        bool joint_states_received_ = false;          // true dopo il primo messaggio di stato reale
        double return_motion_duration;         // durata spline di ritorno [s], da YAML (solo !use_sim_time)
        bool arm_motion_active_ = false;
        rclcpp::Time arm_motion_start_time_;
        std::vector<double> arm_motion_start_positions_;
        std::vector<double> arm_motion_target_positions_;
        double forward_motion_duration;
        const double joint_tolerance = 2e-4; // radians
        std::string real_joints_states_topic = "/omni_controller/joints_state";
        std::string real_joints_command_topic = "/omni_controller/joints_reference";
    

        // Timers
        rclcpp::TimerBase::SharedPtr pose_timer;
        rclcpp::TimerBase::SharedPtr control_timer;
        int control_dt;
        int trajectory_time_ms;
        int trajectory_dt;

        // Timers for laser activation
        rclcpp::TimerBase::SharedPtr first_laser_timer;
        rclcpp::TimerBase::SharedPtr second_laser_timer;

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
        double lambda_IBVS;
        bool variable_gain;
        double lambda_0;
        double lambda_inf;
        double lambda_prime_0;
        double control_threshold;

        // Messages
        control_msgs::action::FollowJointTrajectory::Goal goal_msg;
        rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions goal_options;

        // Subscriptions and publishers
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
        rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr joints_client;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr simulated_laser_pub;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr real_laser_pub;
        rclcpp::Publisher<pi3hat_moteus_int_msgs::msg::JointsCommand>::SharedPtr command_publisher_;
        rclcpp::Subscription<pi3hat_moteus_int_msgs::msg::JointsStates>::SharedPtr states_subscription_;

        // Image QoS subscriber
        message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
        message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,sensor_msgs::msg::Image,sensor_msgs::msg::CameraInfo> SyncPolicy;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,sensor_msgs::msg::Image> RealSyncPolicy;
        std::shared_ptr<message_filters::Synchronizer<RealSyncPolicy>> real_sync;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr real_info_sub;

        // TF2 parameters
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        rclcpp::Time last_warning_time;
        uint64_t message_throttle_ms = 1000; // for INFO and WARN messages using THROTTLE

        // Functions
        void declare_and_get_parameters();   
        void send_joint_trajectory(const std::vector<double>& joint_angles);
        bool vectors_are_equal(const std::vector<double>& vec1, const std::vector<double>& vec2);
        void activate_laser();
        std::vector<double> IBVS_control(Eigen::Vector3d& feature);
        
        // Callbacks
        void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
        void pose_timer_callback();
        void pose_callback(const geometry_msgs::msg::TransformStamped msg);
        void result_callback(const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result);
        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr rgb_msg,
                            const sensor_msgs::msg::Image::ConstSharedPtr depth_msg,
                            const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg);
        void control_callback();
        void real_image_callback(const sensor_msgs::msg::Image::ConstSharedPtr rgb_msg,
                    const sensor_msgs::msg::Image::ConstSharedPtr depth_msg);
        void real_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg);
        void real_states_callback(const pi3hat_moteus_int_msgs::msg::JointsStates::SharedPtr msg);

        // Other packages
        std::unique_ptr<WeedDetector> weed_detector;
        std::string model_path = "/home/simone/Documenti/Uni/arm_mazzolini/src/arm_mazzolini/models/YOLO_networks/best_2026_05_01.onnx";
        std::unique_ptr<ArmKinematic> arm_kinematic;
   
    };

} // namespace arm_mazzolini

