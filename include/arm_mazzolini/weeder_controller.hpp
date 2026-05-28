#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/callback_group.hpp"
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
#include <mutex>
#include <unordered_map>

#include "arm_mazzolini/sphere_detector.hpp"
#include "arm_mazzolini/weed_detector.hpp"
#include "arm_mazzolini/arm_kinematic.hpp"
#include "arm_mazzolini/shared_classes.hpp"
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

        // ─────────────────────────────────────────────────────────────────
        // Callback groups for multi-threaded execution
        // ─────────────────────────────────────────────────────────────────
        // Two MutuallyExclusive groups served by a MultiThreadedExecutor:
        //   - control_cb_group_    : control loop, pose timer, joint state
        //                            updates, action result and laser timers.
        //   - perception_cb_group_ : image/info callbacks (detection).
        // Within each group only one callback runs at a time; across groups
        // they can run in parallel, so a slow detection does not block the
        // 15 ms pose timer.
        rclcpp::CallbackGroup::SharedPtr control_cb_group_;
        rclcpp::CallbackGroup::SharedPtr perception_cb_group_;

        // Mutex protecting state shared between the two groups.
        // Protects: controller_status, target_feature, target_camera_position,
        // target_position, joint_states, joint_states_received_.
        // All other members are accessed only inside a single group and
        // therefore need no explicit lock.
        std::mutex state_mutex_;

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

        inline static const std::unordered_map<std::string, DetectorType> detector_map = {
            {"HSV_red",       DetectorType::HSV_red},
            {"ExG_threshold", DetectorType::ExG_threshold},
            {"ExG_Otsu",      DetectorType::ExG_Otsu},
            {"ExGR",          DetectorType::ExGR},
            {"YOLO",          DetectorType::YOLO}
        };
        DetectorType detector_type;

        // Image parameters
        bool camera_initialized = false;
        std::vector<Eigen::Vector3d> target_buffer;
        size_t image_buffer_size;
        int detection_period_ms;
        rclcpp::Time last_detection_time;
        int roi_size;
        int morph_kernel_size;
        int depth_roi_size;
        float confidence_threshold;
        int CUDA_device_id;
        std::string detector_type_string;

        // Mobile robot pose (and threshold)
        Eigen::Isometry3d old_pose;
        Eigen::Isometry3d new_pose;
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
        bool joint_states_received_ = false;
        double return_motion_duration;
        bool arm_motion_active_ = false;
        rclcpp::Time arm_motion_start_time_;
        std::vector<double> arm_motion_start_positions_;
        std::vector<double> arm_motion_target_positions_;
        double forward_motion_duration;
        const double joint_tolerance = 1e-4;
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
        Eigen::Vector3d target_feature = Eigen::Vector3d::Zero();
        Eigen::Vector3d target_position = Eigen::Vector3d::Zero();
        Eigen::Vector3d target_camera_position = Eigen::Vector3d::Zero();
        Eigen::Vector3d desired_feature;
        Eigen::Vector3d desired_position;
        Eigen::Vector3d e_s_filtered = Eigen::Vector3d::Zero();
        Eigen::Vector3d e_p_filtered = Eigen::Vector3d::Zero();
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
        uint64_t message_throttle_ms = 1000;

        // Functions
        void declare_and_get_parameters();
        void send_joint_trajectory(const std::vector<double>& joint_angles);
        bool vectors_are_equal(const std::vector<double>& vec1, const std::vector<double>& vec2);
        void activate_laser();

        // IBVS_control now takes joint_states as a parameter so the caller
        // can pass a local snapshot taken under state_mutex_, avoiding
        // unprotected reads of the shared joint_states member.
        std::vector<double> IBVS_control(Eigen::Vector3d& feature,
                                          const std::vector<double>& joint_states_snapshot);

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

        // Detector (Strategy pattern: SphereDetector or WeedDetector)
        std::unique_ptr<StandardDetector> detector;
        std::string model_path = "/home/simone/Documenti/Uni/arm_mazzolini/src/arm_mazzolini/models/YOLO_networks/best_2026_05_01.onnx";
        std::unique_ptr<ArmKinematic> arm_kinematic;
    };

} // namespace arm_mazzolini
