#include "arm_mazzolini/weeder_controller.hpp"

namespace arm_mazzolini
{
    WeederController::WeederController() : Node("weeder_controller"),
        sync_(std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), rgb_sub_, depth_sub_, info_sub_))
    {
        // Parameters
        declare_and_get_parameters();

        // Arm kinematic initialization
        arm_kinematic = std::make_unique<ArmKinematic>(l1, l2);

        // Image detector initialization
        sphere_detector = std::make_unique<SphereDetector>();

        // Image subscriptions
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        rgb_sub_.subscribe(this, camera_rgb_topic, qos.get_rmw_qos_profile());
        depth_sub_.subscribe(this, camera_depth_topic, qos.get_rmw_qos_profile());
        info_sub_.subscribe(this, camera_info_topic, qos.get_rmw_qos_profile());
        sync_->registerCallback(std::bind(&WeederController::image_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        // TF2 listener
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        timer = rclcpp::create_timer(
            this,
            this->get_clock(),
            rclcpp::Duration(std::chrono::milliseconds(tf_callback_period_ms)),
            std::bind(&WeederController::timer_callback, this)
        );
        last_warning_time = this->now();

        // Action Client
        joints_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            this,
            "/joint_trajectory_controller/follow_joint_trajectory"
        );
        if (joints_client->wait_for_action_server(std::chrono::seconds(10)) == false) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            RCLCPP_INFO(this->get_logger(), "Shutting down...");
            rclcpp::shutdown();
        }
        goal_options.result_callback = std::bind(&WeederController::result_callback, this, std::placeholders::_1);
        goal_msg.trajectory.joint_names = joint_names;

        // Publisher
        laser_pub = this->create_publisher<geometry_msgs::msg::Point>("/lasered_position", 10);

        // Initial status
        new_pose.setIdentity();
        old_pose.setIdentity();
        controller_status = ControllerStatus::NO_TARGET;

    }

    void WeederController::declare_and_get_parameters()
    {
        try {
            this->declare_parameter("link1_length", double());
            this->declare_parameter("link2_length", double());  
            this->declare_parameter("tf_callback_period", int());
            this->declare_parameter("image_buffer_size", int());
            this->declare_parameter("frames_delay", int());
            this->declare_parameter("camera_rgb_topic", std::string());
            this->declare_parameter("camera_depth_topic", std::string());
            this->declare_parameter("camera_info_topic", std::string());
        

            l1 = this->get_parameter("link1_length").as_double();
            l2 = this->get_parameter("link2_length").as_double();
            tf_callback_period_ms = this->get_parameter("tf_callback_period").as_int();
            auto img_buffer_size = this->get_parameter("image_buffer_size").as_int();
            image_buffer_size = static_cast<size_t>(img_buffer_size);
            frames_delay = this->get_parameter("frames_delay").as_int();

            camera_rgb_topic = this->get_parameter("camera_rgb_topic").as_string();
            camera_depth_topic = this->get_parameter("camera_depth_topic").as_string();
            camera_info_topic = this->get_parameter("camera_info_topic").as_string();

            // ========================== Forse questi non servono più ============================================
            // this->declare_parameter("base_frame_transform.translation", std::vector<double>());
            // this->declare_parameter("base_frame_transform.rotation", std::vector<double>());
            // RCLCPP_INFO(this->get_logger(), "Link lengths: l1 = %.3f, l2 = %.3f", l1, l2);
            // const auto trans_vec = this->get_parameter("base_frame_transform.translation").as_double_array();
            // const auto rot_vec = this->get_parameter("base_frame_transform.rotation").as_double_array();

            // for (size_t i = 0; i < 3; ++i) {
            //     translation[i] = trans_vec[i];
            //     rotation[i] = rot_vec[i];
            // }
        } 
        catch(const rclcpp::exceptions::InvalidParameterTypeException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Error in params declaration: %s", ex.what());
            RCLCPP_INFO(this->get_logger(), "Shutting down...");
            rclcpp::shutdown();
        }
    }

    void WeederController::image_callback(
        const sensor_msgs::msg::Image::SharedPtr rgb_msg,
        const sensor_msgs::msg::Image::SharedPtr depth_msg,
        const sensor_msgs::msg::CameraInfo::SharedPtr info_msg)
    {
        switch (controller_status) {
            case ControllerStatus::NO_TARGET:
                {
                    if(current_frame_index < frames_delay) {
                        // Not processing every message because unnecessary
                        current_frame_index++;
                        return;
                    }
                    else{
                        current_frame_index = 0;

                        Eigen::Vector3d actual_position;
                        
                        if(!sphere_detector->DetectSphere(rgb_msg, depth_msg, actual_position)) {
                            RCLCPP_INFO(this->get_logger(), "No target detected");
                            return;
                        }
                        else {
                            controller_status = ControllerStatus::HAS_TARGET;
                        }
                    }
                    // no break
                }

            case ControllerStatus::HAS_TARGET:
                {
                    Eigen::Vector3d actual_position;
                    if(!sphere_detector->DetectSphere(rgb_msg, depth_msg, actual_position)) {
                        RCLCPP_WARN(this->get_logger(), "Target lost!");
                        target_buffer.clear();
                        controller_status = ControllerStatus::NO_TARGET;
                        return;
                    }
                    else {
                        target_buffer.push_back(actual_position);
                        RCLCPP_INFO(this->get_logger(), "Target detected at (%.3f, %.3f, %.3f)", actual_position.x(), actual_position.y(), actual_position.z());
                        
                        if(target_buffer.size() >= image_buffer_size) {
                            // Compute temporal average and send trajectory
                            auto target_relative_position = Eigen::Vector3d::Zero();
                            for(const auto& pos : target_buffer) {
                                target_relative_position += pos;
                            }
                            
                            target_relative_position /= static_cast<double>(image_buffer_size);
                            target_buffer.clear();

                            try {
                                geometry_msgs::msg::TransformStamped camera_pose = tf_buffer->lookupTransform("arm_base_link", "camera_optical_frame", tf2::TimePointZero);
                                Eigen::Isometry3d camera_relative_pose = tf2::transformToEigen(camera_pose);
                                target_position = camera_relative_pose * target_relative_position;
                                controller_status = ControllerStatus::ARM_MOVING;
                                // store target position and use it later, DO NOT MOVE NOW!
                            }
                            catch (tf2::TransformException &ex) {
                                RCLCPP_WARN(this->get_logger(), "Could not transform camera_optical_frame to arm_base_link: %s", ex.what());
                            }
                                
                    }
                }
                break;
            }

            case ControllerStatus::ARM_MOVING:
            {
                return;
            }

            case ControllerStatus::POSITIONING:
            {
                // TODO: passare a 2 1/2 D
                return;
            }

            case ControllerStatus::LASERING:
            {
                return;
            }
        }

        // TODO: altro da aggiungere?
    }

    
    void WeederController::timer_callback()
    {
        // check if transform is available
        if (!tf_buffer->canTransform("odom", "base_link", tf2::TimePointZero)) {
            RCLCPP_WARN(this->get_logger(), "Transform odom->base_link not available yet");
            timer->reset(); 
            return;
        }
        
        try {
            geometry_msgs::msg::TransformStamped pose_message = 
            tf_buffer->lookupTransform("odom", "base_link", tf2::TimePointZero);
            pose_callback(pose_message);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform odom to base_link: %s", ex.what());
            return;
        }
    }

    void WeederController::pose_callback(geometry_msgs::msg::TransformStamped msg)
    {
        // ------------------- TODO: What do I do of header? ----------------------
        new_pose = tf2::transformToEigen(msg); // convertion out of switch case because i could change message type

        switch (controller_status){

            case ControllerStatus::NO_TARGET:
            {
                // Does it needs to do something here? 
                // Switch to HAS_TARGET is set in image_callback
                break;
            }

            case ControllerStatus::HAS_TARGET:
            {
                if (!new_pose.isApprox(old_pose, pose_threshold)) {
                    RCLCPP_INFO(this->get_logger(), "STOP!");
                    target_buffer.clear();
                    // TODO: Altro?
                }

                break;
            }

            case ControllerStatus::ARM_MOVING:
            {
                if (!new_pose.isApprox(old_pose, pose_threshold)) {
                    RCLCPP_INFO(this->get_logger(), "STOP!! Arm moving");
                    target_buffer.clear();
                    // TODO: Altro?
                }
                else {
                    ErrorType error_type;
                    std::vector<double> joint_angles;
                    if (arm_kinematic->computeIK(target_position, joint_angles, error_type)){
                        RCLCPP_INFO(this->get_logger(), "Arm moving to target");
                        send_joint_trajectory(joint_angles);
                        controller_status = ControllerStatus::POSITIONING;
                    }
                    else {
                        // TODO: check every case
                        switch (error_type) {
                            case ErrorType::TARGET_EMPTY:
                            {
                                RCLCPP_WARN(this->get_logger(), "Sent empty joint angles, check for error");
                                break;
                            }
                            case ErrorType::TARGET_TOO_FAR:
                            {
                                RCLCPP_INFO(this->get_logger(), "Get closer, target out of reach.");
                                // TODO: print position to check if calculations are correct
                                break;
                            }
                            case ErrorType::EXCLUSION_ZONE:
                            {
                                RCLCPP_INFO(this->get_logger(), "Target unreachable, move around obstacles");
                                break;
                            }
                        }
                    }
                }
                break;
            }
            case ControllerStatus::POSITIONING:
            {
                if (!new_pose.isApprox(old_pose, pose_threshold)) {
                    RCLCPP_INFO(this->get_logger(), "Mobile robot moved during POSITIONING");
                    // TODO: abort arm movement
                }

                break;
            }

            case ControllerStatus::LASERING:
            {
                if (!new_pose.isApprox(old_pose, pose_threshold)) {
                    // TODO: cosa si fa qui? è troppo sensibile
                }

                break;
            }

        }

    }

    void WeederController::send_joint_trajectory(const std::vector<double>& joint_angles)
    {
        goal_msg.trajectory.points.clear();
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = joint_angles;
        point.time_from_start = rclcpp::Duration(std::chrono::milliseconds(trajectory_time_ms));
        goal_msg.trajectory.points.push_back(point);

        joints_client->async_send_goal(goal_msg, goal_options);
    }

    void WeederController::result_callback(const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
            {
                RCLCPP_INFO(this->get_logger(), "BYE BYE PLANT!");

                controller_status = ControllerStatus::LASERING; // TODO: check what to do in LASERING status
                geometry_msgs::msg::Point lasered_position = tf2::toMsg(target_position);
                laser_pub->publish(lasered_position);
                // target_position.setZero(); // not safe doing it here
                controller_status = ControllerStatus::NO_TARGET;
                return;
            }
                // TODO: handle other cases
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Joint trajectory execution aborted.");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Joint trajectory execution canceled.");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
                return;
        }

    }
} // namespace arm mazzolini

