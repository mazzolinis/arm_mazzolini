#include "arm_mazzolini/weeder_controller_with_YOLO.hpp"


namespace arm_mazzolini
{
    WeederControllerWithYolo::WeederControllerWithYolo() : Node("weeder_controller_with_YOLO")
    {
        // Parameters
        declare_and_get_parameters();

        // Arm kinematic initialization
        arm_kinematic = std::make_unique<ArmKinematic>(l1, l2);

        // Image detector initialization
        weed_detector = std::make_unique<WeedDetector>(model_path, confidence_threshold, depth_roi_size, CUDA_device_id);
        desired_position = weed_detector->PBTargetPosition(desired_feature);

        if (!use_sim_time) {

            // Image subscriptions — real cameras publish with best-effort/sensor-data QoS
            auto qos = rclcpp::SensorDataQoS();
            rgb_sub_.subscribe(this, camera_rgb_topic, qos.get_rmw_qos_profile());
            depth_sub_.subscribe(this, camera_depth_topic, qos.get_rmw_qos_profile());
            real_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                camera_info_topic,
                rclcpp::SensorDataQoS(),
                std::bind(&WeederControllerWithYolo::real_info_callback, this, std::placeholders::_1)
            );
            real_sync = std::make_shared<message_filters::Synchronizer<RealSyncPolicy>>(RealSyncPolicy(10), rgb_sub_, depth_sub_);
            real_sync->setMaxIntervalDuration(rclcpp::Duration(0, 100000000));  // 100 ms
            real_sync->registerCallback(std::bind(&WeederControllerWithYolo::real_image_callback, this, std::placeholders::_1, std::placeholders::_2));

            // Joint states subscription
            states_subscription_ = this->create_subscription<pi3hat_moteus_int_msgs::msg::JointsStates>(
                real_joints_states_topic,
                10,
                std::bind(&WeederControllerWithYolo::real_states_callback, this, std::placeholders::_1)
            );

            message_throttle_ms = message_throttle_ms * 5;

            // Command publisher
            command_publisher_ = this->create_publisher<pi3hat_moteus_int_msgs::msg::JointsCommand>(real_joints_command_topic, 10);
            // Laser publisher
            real_laser_pub = this->create_publisher<std_msgs::msg::Empty>("/shoot", 10);

        }
        else {

            // Image subscriptions
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
            rgb_sub_.subscribe(this, camera_rgb_topic, qos.get_rmw_qos_profile());
            depth_sub_.subscribe(this, camera_depth_topic, qos.get_rmw_qos_profile());
            info_sub_.subscribe(this, camera_info_topic, qos.get_rmw_qos_profile());
            sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), rgb_sub_, depth_sub_, info_sub_);
            sync_->registerCallback(std::bind(&WeederControllerWithYolo::image_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

            // Joint states subscription
            joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 
                rclcpp::SensorDataQoS(),
                std::bind(&WeederControllerWithYolo::joint_states_callback, this, std::placeholders::_1)
            );

            // Joint trajectory controller action client
            joints_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
                this,
                "/joint_trajectory_controller/follow_joint_trajectory"
            );
            if (joints_client->wait_for_action_server(std::chrono::seconds(10)) == false) {
                RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                RCLCPP_ERROR(this->get_logger(), "Shutting down...");
                rclcpp::shutdown();
            }
            goal_options.result_callback = std::bind(&WeederControllerWithYolo::result_callback, this, std::placeholders::_1);
            goal_msg.trajectory.joint_names = joint_names;

            // Joint trajectory publisher (for POSITIONING (visual servoing))
            trajectory_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);

            // Laser publisher
            simulated_laser_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/lasered_position", 10);
        }

        // TF2 listener
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        pose_timer = rclcpp::create_timer(
            this,
            this->get_clock(),
            rclcpp::Duration(std::chrono::milliseconds(tf_callback_period_ms)),
            std::bind(&WeederControllerWithYolo::pose_timer_callback, this)
        );
        last_warning_time = this->now();

        // Timer for visual servoing
        control_timer = rclcpp::create_timer(
            this,
            this->get_clock(),
            rclcpp::Duration(std::chrono::milliseconds(control_dt)),
            std::bind(&WeederControllerWithYolo::control_callback, this)
        );
        last_detection_time = this->now();

        // Initial status
        new_pose.setIdentity();
        old_pose.setIdentity();
        controller_status = ControllerStatus::NO_TARGET;

        RCLCPP_DEBUG(this->get_logger(), "SETUP COMPLETE");
    }

    void WeederControllerWithYolo::declare_and_get_parameters()
    {
        try {
            // Declarations
            this->declare_parameter("link1_length", double());
            this->declare_parameter("link2_length", double());  
            this->declare_parameter("tf_callback_period", int());
            this->declare_parameter("image_buffer_size", int());
            this->declare_parameter("detection_period_ms", int());
            this->declare_parameter("camera_rgb_topic", std::string());
            this->declare_parameter("camera_depth_topic", std::string());
            this->declare_parameter("camera_info_topic", std::string());
            this->declare_parameter("confidence_threshold", double());
            this->declare_parameter("depth_roi_size", int());
            this->declare_parameter("CUDA_device_id", int());    
            this->declare_parameter("trajectory_time_ms", 250);
            this->declare_parameter("visual_servoing.filtering", true);
            // this->declare_parameter("visual_servoing.alpha", std::vector<double>());
            this->declare_parameter("visual_servoing.beta", std::vector<double>());
            this->declare_parameter("visual_servoing.lambda_IBVS", double());
            this->declare_parameter("visual_servoing.variable_gain", false);
            this->declare_parameter("visual_servoing.lambda_0", double());
            this->declare_parameter("visual_servoing.lambda_inf", double());
            this->declare_parameter("visual_servoing.lambda_prime_0", double());
            this->declare_parameter("visual_servoing.controller_period_ms", int());
            this->declare_parameter("visual_servoing.smoothing", double());
            this->declare_parameter("visual_servoing.control_threshold", 0.005);
            this->declare_parameter("visual_servoing.desired_feature", std::vector<double>());
            this->declare_parameter("return_motion_duration", 10.0);
            this->declare_parameter("forward_motion_duration", 2.0);
            this->declare_parameter("kp_scale", std::vector<double>{0.05, 0.05});
            this->declare_parameter("kd_scale", std::vector<double>{0.05, 0.05});


            // First get simulation flag
            use_sim_time = this->get_parameter("use_sim_time").as_bool();

            // Arm lengths
            l1 = this->get_parameter("link1_length").as_double();
            l2 = this->get_parameter("link2_length").as_double();

            // Camera topic names
            camera_rgb_topic = this->get_parameter("camera_rgb_topic").as_string();
            camera_depth_topic = this->get_parameter("camera_depth_topic").as_string();
            camera_info_topic = this->get_parameter("camera_info_topic").as_string();

            // Times and delays
            tf_callback_period_ms = this->get_parameter("tf_callback_period").as_int();
            auto img_buffer_size = this->get_parameter("image_buffer_size").as_int();
            image_buffer_size = static_cast<size_t>(img_buffer_size);
            detection_period_ms = this->get_parameter("detection_period_ms").as_int();
            trajectory_time_ms = this->get_parameter("trajectory_time_ms").as_int();

            // Camera parameters
            confidence_threshold = this->get_parameter("confidence_threshold").as_double(); 
            depth_roi_size = this->get_parameter("depth_roi_size").as_int();
            CUDA_device_id = this->get_parameter("CUDA_device_id").as_int();

            // Visual servoing parameters
            filtering = this->get_parameter("visual_servoing.filtering").as_bool();
            // alpha = Eigen::Vector3d::Map(this->get_parameter("visual_servoing.alpha").as_double_array().data());
            beta = Eigen::Vector3d::Map(this->get_parameter("visual_servoing.beta").as_double_array().data());

            lambda_IBVS = this->get_parameter("visual_servoing.lambda_IBVS").as_double();
            variable_gain = this->get_parameter("visual_servoing.variable_gain").as_bool();
            lambda_0 = this->get_parameter("visual_servoing.lambda_0").as_double();
            lambda_inf = this->get_parameter("visual_servoing.lambda_inf").as_double();
            lambda_prime_0 = this->get_parameter("visual_servoing.lambda_prime_0").as_double();

            control_dt = this->get_parameter("visual_servoing.controller_period_ms").as_int();
            double smoothing = this->get_parameter("visual_servoing.smoothing").as_double();
            trajectory_dt = static_cast<int>(control_dt * smoothing);
            control_threshold = this->get_parameter("visual_servoing.control_threshold").as_double();
            desired_feature = Eigen::Vector3d::Map(this->get_parameter("visual_servoing.desired_feature").as_double_array().data());
            return_motion_duration = this->get_parameter("return_motion_duration").as_double();
            forward_motion_duration = this->get_parameter("forward_motion_duration").as_double();
            kp_scale = this->get_parameter("kp_scale").as_double_array();
            kd_scale = this->get_parameter("kd_scale").as_double_array();
        } 

        catch(const rclcpp::exceptions::InvalidParameterTypeException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Error in params declaration: %s", ex.what());
            RCLCPP_ERROR(this->get_logger(), "Shutting down...");
            rclcpp::shutdown();
        }
    }

    void WeederControllerWithYolo::image_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr depth_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg)
    {
        if (!camera_initialized) {
            // Hypothesis: camera info are constant
            weed_detector->SetCameraInfo(info_msg);
            camera_initialized = true;
            RCLCPP_DEBUG(this->get_logger(), "Camera initialized");
        }

        if (controller_status == ControllerStatus::LASERING) {
            return;
        }

        auto now = this->now();
        double elapsed_ms = (now - last_detection_time).seconds() * 1000.0;
        if (elapsed_ms < detection_period_ms)
            return;
        last_detection_time = now;

        switch (controller_status) {
            case ControllerStatus::NO_TARGET:
                {
                    // NO DELAY: I already have detection_period_ms to throttle detection
                    Eigen::Vector3d actual_position;
                    RCLCPP_DEBUG(this->get_logger(), "Detecting target...");
                    
                    if(!weed_detector->PBTargetPosition(rgb_msg, depth_msg, actual_position)) {
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *(this->get_clock()), message_throttle_ms, "No target detected");
                        return;
                    }
                    else {
                        controller_status = ControllerStatus::HAS_TARGET;
                        return_motion_active_ = false;
                    }
                    // no break
                }

            case ControllerStatus::HAS_TARGET:
                {
                    Eigen::Vector3d actual_position;

                    // No target = clear buffer and go back to NO_TARGET
                    if(!weed_detector->PBTargetPosition(rgb_msg, depth_msg, actual_position)) {
                        RCLCPP_INFO(this->get_logger(), "Target lost!");
                        target_buffer.clear();
                        controller_status = ControllerStatus::NO_TARGET;
                        return;
                    }
                    // Target detected = add it to buffer
                    else {
                        target_buffer.push_back(actual_position);
                        RCLCPP_DEBUG(this->get_logger(), "Target detected at [%.3f, %.3f, %.3f] in camera frame", actual_position[0], actual_position[1], actual_position[2]);

                        // Buffer full = compute average and move to that position
                        if(target_buffer.size() >= image_buffer_size) {
                            target_camera_position.setZero();
                            for(const auto& pos : target_buffer) {
                                target_camera_position += pos;
                            }
                            
                            target_camera_position /= static_cast<double>(image_buffer_size);
                            target_buffer.clear();
                            RCLCPP_INFO(this->get_logger(), "Target found. STOP!");

                            try {
                                
                                geometry_msgs::msg::TransformStamped camera_pose = tf_buffer->lookupTransform("kinematic_link", "camera_kinematic", tf2::TimePointZero);
                                Eigen::Isometry3d camera_to_kinematic = tf2::transformToEigen(camera_pose);
                                target_position = camera_to_kinematic * target_camera_position;
                                controller_status = ControllerStatus::ARM_MOVING;

                                // Debug print:
                                geometry_msgs::msg::TransformStamped base_to_kinematic = tf_buffer->lookupTransform("kinematic_link", "arm_base_link", tf2::TimePointZero);
                                Eigen::Isometry3d base_to_kinematic_eigen = tf2::transformToEigen(base_to_kinematic);
                                geometry_msgs::msg::TransformStamped base_to_camera = tf_buffer->lookupTransform("arm_base_link", "camera_kinematic", tf2::TimePointZero);
                                Eigen::Isometry3d base_to_camera_eigen = tf2::transformToEigen(base_to_camera);
                                Eigen::Vector3d target_in_arm_base = base_to_kinematic_eigen * target_position;

                                RCLCPP_DEBUG_STREAM(this->get_logger(), "====================== DEBUG INVERSE KINEMATIC: ======================");
                                RCLCPP_DEBUG_STREAM(this->get_logger(), "Joint states: " << joint_states[0] << ", " << joint_states[1]);
                                RCLCPP_DEBUG_STREAM(this->get_logger(), "Target in camera frame: " << std::endl << target_camera_position);
                                RCLCPP_DEBUG_STREAM(this->get_logger(), "Base to kinematic rotation: " << std::endl << base_to_kinematic_eigen.rotation());
                                RCLCPP_DEBUG_STREAM(this->get_logger(), "Base to kinematic translation: " << std::endl << base_to_kinematic_eigen.translation());
                                RCLCPP_DEBUG_STREAM(this->get_logger(), "Base to camera rotation: " << std::endl << base_to_camera_eigen.rotation());
                                RCLCPP_DEBUG_STREAM(this->get_logger(), "Base to camera translation: " << std::endl << base_to_camera_eigen.translation());
                                RCLCPP_DEBUG_STREAM(this->get_logger(), "Target in arm base frame: " << std::endl << target_in_arm_base);
                                RCLCPP_DEBUG_STREAM(this->get_logger(), "====================================================================");
                            }
                            catch (tf2::TransformException &ex) {
                                RCLCPP_WARN_THROTTLE(
                                    this->get_logger(),
                                    *(this->get_clock()),
                                    message_throttle_ms,
                                    "Could not transform camera_kinematic to kinematic_link: %s", ex.what()
                                );
                            }

                    }
                }
                break;
            }

            case ControllerStatus::ARM_MOVING:
            {
                break;
            }

            case ControllerStatus::POSITIONING:
            {
                // Here I just store position for visual servoing, control is done in visual_control function (2nd timer)
                if(!weed_detector->IBTargetPosition(rgb_msg, depth_msg, target_feature)) {
                    RCLCPP_INFO(this->get_logger(), "Target lost!");
                    target_buffer.clear();
                    controller_status = ControllerStatus::NO_TARGET;
                    return;
                }
                else {
                    target_camera_position = weed_detector->PBTargetPosition(target_feature);
                }
                break;
            }

            case ControllerStatus::LASERING:
            {
                break;
            }
        }

        // TODO: altro da aggiungere?
    }

    void WeederControllerWithYolo::real_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg)
    {
        if (!camera_initialized) {
            weed_detector->SetCameraInfo(info_msg);
            camera_initialized = true;
            RCLCPP_INFO(this->get_logger(), "Camera info received, initialized Detector");
        }
    }

    void WeederControllerWithYolo::real_image_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr depth_msg)
    {
        if (!camera_initialized) {
            RCLCPP_WARN(this->get_logger(), "Camera info not received yet, cannot initialize Detector");
            return;
        }

        if (controller_status == ControllerStatus::LASERING) {
            return;
        }

        auto now = this->now();
        double elapsed_ms = (now - last_detection_time).seconds() * 1000.0;
        if (elapsed_ms < detection_period_ms)
            return;
        last_detection_time = now;

        switch (controller_status) {
            case ControllerStatus::NO_TARGET:
                {
                    // NO DELAY: I already have detection_period_ms to throttle detection
                    Eigen::Vector3d actual_position;
                    RCLCPP_DEBUG(this->get_logger(), "Detecting target...");

                    if(!weed_detector->PBTargetPosition(rgb_msg, depth_msg, actual_position)) {
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *(this->get_clock()), message_throttle_ms, "No target detected");
                        return;
                    }
                    else {
                        controller_status = ControllerStatus::HAS_TARGET;
                        return_motion_active_ = false;
                    }
                    // no break
                }

            case ControllerStatus::HAS_TARGET:
                {
                    Eigen::Vector3d actual_position;

                    // No target = clear buffer and go back to NO_TARGET
                    if(!weed_detector->PBTargetPosition(rgb_msg, depth_msg, actual_position)) {
                        RCLCPP_INFO(this->get_logger(), "Target lost!");
                        target_buffer.clear();
                        controller_status = ControllerStatus::NO_TARGET;
                        return;
                    }
                    // Target detected = add it to buffer
                    else {
                        target_buffer.push_back(actual_position);
                        RCLCPP_INFO(this->get_logger(), "Target detected at [%.3f, %.3f, %.3f] in camera frame", actual_position[0], actual_position[1], actual_position[2]);
                        
                        // Buffer full = compute average and move to that position
                        if(target_buffer.size() >= image_buffer_size) {
                            target_camera_position.setZero();
                            for(const auto& pos : target_buffer) {
                                target_camera_position += pos;
                            }

                            target_camera_position /= static_cast<double>(image_buffer_size);
                            target_buffer.clear();

                            try {
                                geometry_msgs::msg::TransformStamped camera_pose = tf_buffer->lookupTransform("kinematic_link", "camera_kinematic", tf2::TimePointZero);
                                Eigen::Isometry3d camera_to_kinematic = tf2::transformToEigen(camera_pose);
                                target_position = camera_to_kinematic * target_camera_position;
                                controller_status = ControllerStatus::ARM_MOVING;

                                // Debug print:
                                geometry_msgs::msg::TransformStamped base_to_kinematic = tf_buffer->lookupTransform("kinematic_link", "arm_base_link", tf2::TimePointZero);
                                Eigen::Isometry3d base_to_kinematic_eigen = tf2::transformToEigen(base_to_kinematic);
                                geometry_msgs::msg::TransformStamped base_to_camera = tf_buffer->lookupTransform("arm_base_link", "camera_kinematic", tf2::TimePointZero);
                                Eigen::Isometry3d base_to_camera_eigen = tf2::transformToEigen(base_to_camera);
                                Eigen::Vector3d target_in_arm_base = base_to_kinematic_eigen * target_position;

                                RCLCPP_DEBUG_STREAM(this->get_logger(), "====================== DEBUG INVERSE KINEMATIC: ======================");
                                RCLCPP_DEBUG_STREAM(this->get_logger(), "Joint states: " << joint_states[0] << ", " << joint_states[1]);
                                RCLCPP_DEBUG_STREAM(this->get_logger(), "Target in camera frame: " << std::endl << target_camera_position);
                                RCLCPP_DEBUG_STREAM(this->get_logger(), "Base to kinematic rotation: " << std::endl << base_to_kinematic_eigen.rotation());
                                RCLCPP_DEBUG_STREAM(this->get_logger(), "Base to kinematic translation: " << std::endl << base_to_kinematic_eigen.translation());
                                RCLCPP_DEBUG_STREAM(this->get_logger(), "Base to camera rotation: " << std::endl << base_to_camera_eigen.rotation());
                                RCLCPP_DEBUG_STREAM(this->get_logger(), "Base to camera translation: " << std::endl << base_to_camera_eigen.translation());
                                RCLCPP_DEBUG_STREAM(this->get_logger(), "Target in arm base frame: " << std::endl << target_in_arm_base);
                                RCLCPP_DEBUG_STREAM(this->get_logger(), "====================================================================");
                            }
                            catch (tf2::TransformException &ex) {
                                RCLCPP_WARN_THROTTLE(
                                    this->get_logger(),
                                    *(this->get_clock()),
                                    message_throttle_ms,
                                    "Could not transform camera_kinematic to kinematic_link: %s", ex.what()
                                );
                            }

                    }
                }
                break;
            }

            case ControllerStatus::ARM_MOVING:
            {
                break;
            }

            case ControllerStatus::POSITIONING:
            {
                // Here I just store position for visual servoing, control is done in visual_control function (2nd timer)
                if(!weed_detector->IBTargetPosition(rgb_msg, depth_msg, target_feature)) {
                    RCLCPP_INFO(this->get_logger(), "Target lost!");
                    target_buffer.clear();
                    controller_status = ControllerStatus::NO_TARGET;
                    return;
                }
                else {
                    target_camera_position = weed_detector->PBTargetPosition(target_feature);
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *(this->get_clock()), message_throttle_ms,
                        "FOLLOWING TARGET AT POSITION [%.3f, %.3f, %.3f] IN CAMERA FRAME",
                        target_camera_position[0], target_camera_position[1], target_camera_position[2]
                    );
                }
                break;
            }

            case ControllerStatus::LASERING:
            {
                break;
            }
        }

        // TODO: altro da aggiungere?
    }

    void WeederControllerWithYolo::joint_states_callback(
        const sensor_msgs::msg::JointState::SharedPtr msg)
    {

        if (controller_status == ControllerStatus::NO_TARGET ||
            controller_status == ControllerStatus::LASERING) {
            return;
        }
        int idx1 = -1;
        int idx2 = -1;

        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == joint_names[0]) {
                idx1 = static_cast<int>(i);
            }
            else if (msg->name[i] == joint_names[1]) {
                idx2 = static_cast<int>(i);
            }
        }

        // Controllo di sicurezza
        if (idx1 < 0 || idx2 < 0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "joint1 or joint2 not found in /joint_states"
            );
            return;
        }

        // Copia le posizioni
        joint_states[0] = msg->position[idx1];
        joint_states[1] = msg->position[idx2];

        RCLCPP_DEBUG(
            this->get_logger(),
            "Joint states updated: joint1=%.3f joint2=%.3f",
            joint_states[0],
            joint_states[1]
        );
    }

    void WeederControllerWithYolo::real_states_callback(
        const pi3hat_moteus_int_msgs::msg::JointsStates::SharedPtr msg)
    {
        if (controller_status == ControllerStatus::LASERING) {
            return;
        }
        std::vector<double> joint_positions = joint_states; // if something goes wrong, I don't want to lose current joint states, so I work on a copy and then update only if everything is fine
        for(size_t i = 0; i < msg->name.size(); i++) {
            if(std::isfinite(msg->position[i])) {
                // Check if the joint is one of the joints we are interested in
                size_t idx = std::find(joint_names.begin(), joint_names.end(), msg->name[i]) - joint_names.begin();
                if (idx < joint_names.size()) {
                    // Update data
                    joint_positions[idx] = msg->position[i];
                }
            }
            else if(std::isfinite(msg->sec_enc_pos[i])) {
                // Check if the joint is one of the joints we are interested in
                size_t idx = std::find(joint_names.begin(), joint_names.end(), msg->name[i]) - joint_names.begin();
                if (idx < joint_names.size()) {
                    // Update data
                    joint_positions[idx] = msg->sec_enc_pos[i];
                }
            }
            else {
                continue;
            }
        }

        joint_states = joint_positions;
        joint_states_received_ = true;   // motion should not start until we have received joint_states

        RCLCPP_DEBUG(
            this->get_logger(),
            "Real Joint states updated: joint1=%.3f joint2=%.3f",
            joint_states[0],
            joint_states[1]
        );
    }

    
    void WeederControllerWithYolo::pose_timer_callback()
    {
        if (use_sim_time){

            // check if transform is available
            if (!tf_buffer->canTransform("odom", "base_link", tf2::TimePointZero)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *(this->get_clock()), message_throttle_ms, "Transform odom->base_link not available yet");
                pose_timer->reset(); 
                return;
            }
            
            try {
                geometry_msgs::msg::TransformStamped pose_message = 
                tf_buffer->lookupTransform("odom", "base_link", tf2::TimePointZero);
                pose_callback(pose_message);
            }
            catch (tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *(this->get_clock()), message_throttle_ms, "Could not transform odom to base_link: %s", ex.what());
                return;
            }
        }
        else {
            // Forzo la posa ad essere l'identità
            // TODO: trovare il modo per leggere la posa reale
            geometry_msgs::msg::TransformStamped pose_message = geometry_msgs::msg::TransformStamped();
            pose_message.header.stamp = this->get_clock()->now();
            pose_message.header.frame_id = "odom";
            pose_message.child_frame_id = "base_link";
            pose_message.transform.translation.x = 0.0;
            pose_message.transform.translation.y = 0.0;
            pose_message.transform.translation.z = 0.0;
            pose_message.transform.rotation.x = 0.0;
            pose_message.transform.rotation.y = 0.0;
            pose_message.transform.rotation.z = 0.0;
            pose_message.transform.rotation.w = 1.0;
            pose_callback(pose_message);
        }
    }

    void WeederControllerWithYolo::pose_callback(geometry_msgs::msg::TransformStamped msg)
    {
        // ------------------- TODO: What do I do of header? ----------------------
        new_pose = tf2::transformToEigen(msg); // convertion out of switch case because i could change message type

        switch (controller_status){

             case ControllerStatus::NO_TARGET:
            {
                // Send initial position
                // TODO: add scanning movement
                if(use_sim_time) {
                    send_joint_trajectory(initial_joint_values);
                }
                else {
                    // Real hardware: smooth spline back to initial_joint_values.
                    // Duration is configurable from YAML (return_motion_duration).

                    // Wait until we have received real joint states from the hardware,
                    // otherwise return_motion_start_positions_ would be wrong.

                    if (!joint_states_received_) {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *(this->get_clock()),
                            message_throttle_ms, "Waiting for real joint states before return motion...");
                        break;
                    }

                    if (!return_motion_active_) {
                        // Start spline only if we are actually far from the initial pose
                        if (vectors_are_equal(joint_states, initial_joint_values)) {
                            // Already in place: nothing to do
                            return;
                        }
                        return_motion_elapsed_         = 0.0;
                        return_motion_last_tick_       = this->get_clock()->now();
                        return_motion_start_positions_ = joint_states;   // real current position
                        return_motion_active_          = true;
                        RCLCPP_DEBUG(this->get_logger(), "======================================================================");
                        RCLCPP_DEBUG(this->get_logger(),
                            "NEW SPLINE START: [%.3f, %.3f] -> [%.3f, %.3f] in %.1f s",
                            joint_states[0], joint_states[1],
                            initial_joint_values[0], initial_joint_values[1],
                            return_motion_duration);
                        RCLCPP_DEBUG(this->get_logger(), "======================================================================");
                    } else {
                        // Accumulate only the time actually spent in NO_TARGET so that
                        // spurious state transitions (e.g. transient target detections)
                        // do not eat into the 10-second budget.
                        // Cap delta to one timer period: if we were away in another state,
                        // (now - last_tick) would span the entire ARM_MOVING+POSITIONING+LASERING
                        // duration and jump t past 1.0 immediately.
                        auto now = this->get_clock()->now();
                        const double max_delta = static_cast<double>(tf_callback_period_ms) / 1000.0;
                        double delta = std::min((now - return_motion_last_tick_).seconds(), max_delta);
                        return_motion_elapsed_ += delta;
                        return_motion_last_tick_ = now;
                    }

                    double t = std::min(return_motion_elapsed_ / return_motion_duration, 1.0);
                    // // linear interpolation
                    // std::vector<double> interp(initial_joint_values.size());
                    // for (size_t i = 0; i < initial_joint_values.size(); i++) {
                    //     interp[i] = return_motion_start_positions_[i] * (1.0 - t) + initial_joint_values[i] * t;
                    // }

                    // smoothstep (cubic Hermite): zero velocity at both endpoints
                    double s = t * t * (3.0 - 2.0 * t);
                    std::vector<double> interp(initial_joint_values.size());
                    for (size_t i = 0; i < interp.size(); i++) {
                        interp[i] = return_motion_start_positions_[i] * (1.0 - s)
                                  + initial_joint_values[i] * s;
                    }

                    // DEBUG
                    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "SPLINE DEBUG: " << std::endl
                        << "Return motion: elapsed=" << return_motion_elapsed_ << "s t=" << t
                        << " current=[" << joint_states[0] << ", " << joint_states[1] << "]"
                        << " interp=[" << interp[0] << ", " << interp[1] << "]"
                        << " target=[" << initial_joint_values[0] << ", " << initial_joint_values[1] << "]"
                    );

                    send_joint_trajectory(interp);

                    if (t >= 1.0) {
                        return_motion_active_ = false;
                        RCLCPP_INFO(this->get_logger(), "INITIAL POSITION REACHED");
                        send_joint_trajectory(initial_joint_values);
                        // vectors_are_equal will prevent the spline from restarting
                    }
                }
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
                    RCLCPP_INFO(this->get_logger(), "STOP!! Arm moving. Movement aborted.");
                    target_buffer.clear();
                    controller_status = ControllerStatus::HAS_TARGET;
                    if (use_sim_time) {
                        joints_client->async_cancel_all_goals();
                    }
                    else {
                        arm_motion_active_ = false;
                    }
                }
                else {
                    ErrorType error_type;
                    std::vector<double> joint_angles;
                    if (arm_kinematic->computeIK(target_position, joint_angles, error_type)) {
                        if (use_sim_time) {
                            RCLCPP_DEBUG_STREAM(this->get_logger(), "Arm moving to: " << joint_angles[0] << ", " << joint_angles[1]);
                            send_joint_trajectory(joint_angles);
                        }
                        else {
                            if (!arm_motion_active_) {
                                arm_motion_start_time_       = this->get_clock()->now();
                                arm_motion_start_positions_  = joint_states;
                                arm_motion_target_positions_ = joint_angles;
                                arm_motion_active_           = true;
                                RCLCPP_INFO(this->get_logger(),
                                    "Starting arm spline: [%.3f, %.3f] -> [%.3f, %.3f] in %.1f s",
                                    joint_states[0], joint_states[1],
                                    joint_angles[0], joint_angles[1],
                                    forward_motion_duration);
                            }

                            double elapsed = (this->get_clock()->now() - arm_motion_start_time_).seconds();
                
                            double t = std::min(elapsed / forward_motion_duration, 1.0);
                            // // linear interpolation
                            // std::vector<double> interp(arm_motion_target_positions_.size());
                            // for (size_t i = 0; i < interp.size(); i++) {
                            //     interp[i] = arm_motion_start_positions_[i] * (1.0 - t) + arm_motion_target_positions_[i] * t;
                            // }

                            // smoothstep (cubic Hermite): zero velocity at both endpoints    
                            double s = t * t * (3.0 - 2.0 * t);
                            std::vector<double> interp(arm_motion_target_positions_.size());
                            for (size_t i = 0; i < interp.size(); i++) {
                                interp[i] = arm_motion_start_positions_[i] * (1.0 - s)
                                          + arm_motion_target_positions_[i] * s;
                            }

                            send_joint_trajectory(interp);

                            if (t >= 1.0) {
                                arm_motion_active_ = false;
                                controller_status = ControllerStatus::POSITIONING;
                                RCLCPP_INFO(this->get_logger(), "Arm reached target, switching to POSITIONING");
                            }
                        }
                    }
                    else {
                        switch (error_type) {
                            case ErrorType::TARGET_EMPTY:
                                RCLCPP_WARN(this->get_logger(), "Sent empty joint angles, check for error");
                                return;
                            case ErrorType::TARGET_TOO_FAR:
                            {
                                RCLCPP_INFO_THROTTLE(this->get_logger(), *(this->get_clock()), message_throttle_ms, "Get closer, target out of reach.");
                                double r = std::sqrt(target_position.x()*target_position.x() + target_position.y()*target_position.y());
                                RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *(this->get_clock()), message_throttle_ms, "r = " << r << " l1 + l2 = " << (l1 + l2));
                                return;
                            }
                            case ErrorType::EXCLUSION_ZONE:
                                RCLCPP_INFO_THROTTLE(this->get_logger(), *(this->get_clock()), message_throttle_ms, "Target unreachable, move around obstacles");
                                return;
                        }
                    }
                }
                break;
            }

            case ControllerStatus::POSITIONING:
            {
                if (!new_pose.isApprox(old_pose, pose_threshold)) {
                    RCLCPP_INFO(this->get_logger(), "Mobile robot moved during POSITIONING");
                    target_buffer.clear();
                    target_feature.setZero();  
                    target_camera_position.setZero();
                    controller_status = ControllerStatus::NO_TARGET;
                    if (use_sim_time) {
                        joints_client->async_cancel_all_goals();
                    }
                }
                break;
            }

            case ControllerStatus::LASERING:
            {
                if (!use_sim_time && return_motion_active_) {
                    // Return spline: arm moves back to initial position while detection
                    // stays blocked (LASERING keeps image callbacks skipped).
                    auto now = this->get_clock()->now();
                    const double max_delta = static_cast<double>(tf_callback_period_ms) / 1000.0;
                    double delta = std::min((now - return_motion_last_tick_).seconds(), max_delta);
                    return_motion_elapsed_ += delta;
                    return_motion_last_tick_ = now;

                    double t = std::min(return_motion_elapsed_ / return_motion_duration, 1.0);
                    double s = t * t * (3.0 - 2.0 * t);
                    std::vector<double> interp(initial_joint_values.size());
                    for (size_t i = 0; i < interp.size(); i++) {
                        interp[i] = return_motion_start_positions_[i] * (1.0 - s)
                                  + initial_joint_values[i] * s;
                    }
                    send_joint_trajectory(interp);

                    if (t >= 1.0) {
                        return_motion_active_ = false;
                        send_joint_trajectory(initial_joint_values);
                        controller_status = ControllerStatus::NO_TARGET;
                        RCLCPP_INFO(this->get_logger(), "RETURN SPLINE COMPLETE. Ready for new target.");
                    }
                } else if (!new_pose.isApprox(old_pose, pose_threshold)) {
                    controller_status = ControllerStatus::NO_TARGET;
                    RCLCPP_INFO(this->get_logger(), "Mobile robot moved during LASERING");
                }
                break;
            }

        }

        // DO NOT FORGET TO UPDATE POSE
        old_pose = new_pose;
    }

    void WeederControllerWithYolo::send_joint_trajectory(const std::vector<double>& joint_angles)
    {
        if (joint_angles.size() != joint_names.size()) {
            RCLCPP_ERROR(this->get_logger(), "Joint angles size does not match joint names size");
            return;
        }
        if (vectors_are_equal(joint_angles, last_joint_angles)) {
            RCLCPP_DEBUG(this->get_logger(), "Joint angles are the same as last sent, not sending trajectory");
            return;
        }
        last_joint_angles = joint_angles;

        if (use_sim_time) {
            if (controller_status == ControllerStatus::POSITIONING) {
                // For visual servoing I use publisher, not server-action
                trajectory_msgs::msg::JointTrajectory traj;
                traj.joint_names = joint_names;
                trajectory_msgs::msg::JointTrajectoryPoint point;
                point.positions = joint_angles;
                point.time_from_start = rclcpp::Duration(std::chrono::milliseconds(trajectory_dt));
                traj.points.push_back(point);

                trajectory_pub->publish(traj);
            }
            else {
                goal_msg.trajectory.points.clear();
                trajectory_msgs::msg::JointTrajectoryPoint point;
                point.positions = joint_angles;
                point.time_from_start = rclcpp::Duration(std::chrono::milliseconds(trajectory_time_ms));
                goal_msg.trajectory.points.push_back(point);

                joints_client->async_send_goal(goal_msg, goal_options);
            }            
        }
        else {
            pi3hat_moteus_int_msgs::msg::JointsCommand command_msg;
            command_msg.name     = joint_names;
            command_msg.position = joint_angles;
            command_msg.velocity = std::vector<double>(joint_angles.size(), 0.0);   // adding zeros for velocity and effort
            command_msg.effort   = std::vector<double>(joint_angles.size(), 0.0);   
            command_msg.kp_scale = kp_scale;
            command_msg.kd_scale = kd_scale;
            command_publisher_->publish(command_msg);
        }
    }

    void WeederControllerWithYolo::result_callback(const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result)
    {
        if (controller_status == ControllerStatus::LASERING) {
            return;
        }
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
            {
                if (controller_status == ControllerStatus::ARM_MOVING) {
                    controller_status = ControllerStatus::POSITIONING;
                }
                break;
            }
                // TODO: handle other cases
            case rclcpp_action::ResultCode::ABORTED:
                // RCLCPP_ERROR(this->get_logger(), "Joint trajectory execution aborted.");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                // RCLCPP_ERROR(this->get_logger(), "Joint trajectory execution canceled.");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
                break;
        }
    }

    bool WeederControllerWithYolo::vectors_are_equal(const std::vector<double>& vec1, const std::vector<double>& vec2)
    {
        if (vec1.size() != vec2.size()) {
            return false;
        }
        for (size_t i = 0; i < vec1.size(); i++) {
            if (std::abs(vec1[i] - vec2[i]) > joint_tolerance) {
                return false;
            }
            else if (!use_sim_time && std::abs(vec1[i] - vec2[i]) > joint_tolerance * 1.5) {
                return false;
            }
        }
        return true;
    }

    void WeederControllerWithYolo::control_callback()
    {

        if(controller_status != ControllerStatus::POSITIONING) {
            return;
        }
        else if (target_feature.isZero()) {
            RCLCPP_DEBUG(this->get_logger(), "Target feature is zero, skipping control");
            return;
        }
        else {
            double pixel_error = std::sqrt(std::pow(target_feature.x() - desired_feature.x(), 2) + std::pow(target_feature.y() - desired_feature.y(), 2));

            if (pixel_error < control_threshold) {
                activate_laser();
                target_feature.setZero(); 
            }
            else {
                std::vector<double> joint_goal = WeederControllerWithYolo::IBVS_control(target_feature);
                send_joint_trajectory(joint_goal);
            }
        }
    }

    std::vector<double> WeederControllerWithYolo::IBVS_control(Eigen::Vector3d& feature)
    {
        // feature è già normalizzato
        double u = feature.x();
        double v = feature.y();
        // if (feature.z() < 0.3) {
        //     feature.z() = 0.8;
        // }
        double Z = feature.z(); 

        double dt = static_cast<double>(control_dt) / 1000.0;

        // Errore visivo
        Eigen::Vector3d e_s;

        if (filtering) {
            e_s = feature - desired_feature;

            // Smoothing error
            e_s_filtered = beta.cwiseProduct(e_s) + (Eigen::Vector3d(1.0, 1.0, 1.0) - beta).cwiseProduct(e_s_filtered);
            e_s = e_s_filtered;            
        }
        else {
            // Computing error
            e_s = feature - desired_feature;
        }

        // // Interaction matrix simplified for this case (depth camera + only translation)
        // Eigen::Matrix3d L;
        // L <<    -1.0 / Z,   0.0,        u / Z,
        //         0.0,        -1.0 / Z,   v / Z,
        //         0.0,        0.0,        -1.0;

        // // Velocità camera
        // Eigen::Vector3d v_c = - (lambda_IBVS * L.inverse() * e_s);

        // Prova: calcolo direttamente la pseudo-inversa
        Eigen::Matrix3d L_pinv;
        L_pinv <<  -Z,      0.0,     u,
                    0.0,    -Z,      v,
                    0.0,    0.0,    -1.0;

        Eigen::Vector3d v_c;
        
        if (variable_gain) {
            double e_norm = e_s.norm();
            double k = lambda_prime_0 / std::max(lambda_0 - lambda_inf, 1e-3);
            double lambda_eff = (lambda_0 - lambda_inf) * std::exp(-k * e_norm) + lambda_inf;
            v_c = - (lambda_eff * L_pinv * e_s);
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Variable gain: e_norm=" << e_norm << " lambda_eff=" << lambda_eff);
        }
        else {
            v_c = - (lambda_IBVS * L_pinv * e_s);
        }

        // double mu = 0.02;
        // Eigen::Matrix3d L_damped =
        //     L.transpose() * (L * L.transpose()
        //     + mu * mu * Eigen::Matrix3d::Identity()).inverse();

        // Eigen::Vector3d v_c = - (lambda_IBVS * L_damped * e_s);
       
        // Rotazione camera -> EE (costante, da TF)
        Eigen::Isometry3d camera_to_EE;
        try {
            geometry_msgs::msg::TransformStamped camera_pose = tf_buffer->lookupTransform("end_effector", "camera_kinematic", tf2::TimePointZero);
            camera_to_EE = tf2::transformToEigen(camera_pose);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *(this->get_clock()), message_throttle_ms, "Could not transform camera_kinematic to kinematic_link: %s", ex.what());
            return {joint_states[0], joint_states[1]}; // return current joint states to avoid sending empty trajectory
        }
        Eigen::Matrix3d R_ec = camera_to_EE.rotation();

        // Velocità EE
        Eigen::Vector3d v_e = R_ec * v_c;

        // Jacobiano SCARA
        Eigen::Matrix3d J = arm_kinematic->computeScaraJacobian(joint_states);

        // joint velocity
        Eigen::Vector3d q_dot;
        if (use_sim_time) {
            q_dot = J.inverse() * v_e;
        }
        else{
            // Singularity guard: damped least-squares inversion
            const double mu = 0.05;
            Eigen::Matrix3d J_damped = J.transpose() * (J * J.transpose() + mu * mu * Eigen::Matrix3d::Identity()).inverse();
            q_dot = J_damped * v_e;
        }

        // Integrazione
        // Potrei aggiungere un terzo joint state per considerare la focale del laser
        auto q = Eigen::Vector3d(joint_states[0], joint_states[1], 0.0);
        Eigen::Vector3d q_next = q + q_dot * dt;

        RCLCPP_DEBUG(this->get_logger(), "======================DEBUG IBVS======================");
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Feature error (in camera frame): " << e_s.transpose());
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Camera velocity (m/s): " << v_c.transpose());
        RCLCPP_DEBUG_STREAM(this->get_logger(), "EE velocity (m/s): " << v_e.transpose());
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Joint velocity (rad/s): " << q_dot.transpose());
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Current joint angles (rad): " << q.transpose());
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Next joint angles (rad): " << q_next.transpose());
        RCLCPP_DEBUG(this->get_logger(), "=====================================================");

        return {q_next(0), q_next(1)};
    }

    void WeederControllerWithYolo::activate_laser()
    {
        controller_status = ControllerStatus::LASERING;

        int pause_milliseconds;
        if (use_sim_time) {
            pause_milliseconds = 500;
        }
        else {
            // Real hardware: longer pause to allow laser to reach full power
            pause_milliseconds = 10000;
        }

        RCLCPP_INFO(this->get_logger(), "============ ACTIVATING LASER =============");

        if(!use_sim_time) {
            real_laser_pub->publish(std_msgs::msg::Empty());
        }

        first_laser_timer = rclcpp::create_timer(
            this,
            this->get_clock(),
            rclcpp::Duration(std::chrono::milliseconds(pause_milliseconds*8/10)),
            [this]() {
                first_laser_timer->cancel();

                geometry_msgs::msg::PointStamped lasered_position;
                if (use_sim_time) {
                    try {
                        // TODO: remove odom and use global fixed frame
                        geometry_msgs::msg::TransformStamped arm_pose = tf_buffer->lookupTransform("odom", "kinematic_link", tf2::TimePointZero);
                        Eigen::Isometry3d kinematic_to_base = tf2::transformToEigen(arm_pose);
                        Eigen::Vector3d absolute_position = kinematic_to_base * target_position;
                        lasered_position.point = tf2::toMsg(absolute_position);
                        lasered_position.header.stamp = this->now();
                        lasered_position.header.frame_id = "odom";
                        simulated_laser_pub->publish(lasered_position);
                    }
                    catch (tf2::TransformException &ex) {
                        RCLCPP_WARN(this->get_logger(), "Could not transform odom to kinematic_link: %s", ex.what());
                    }
                }
                else {                    
                    real_laser_pub->publish(std_msgs::msg::Empty());
                }
            });

            RCLCPP_INFO(this->get_logger(), "TARGET CLEARED. READY FOR A NEW ONE");

            // second timer to avoid detecting the same plant twice
            second_laser_timer = rclcpp::create_timer(
                this,
                this->get_clock(),
                rclcpp::Duration(std::chrono::milliseconds(pause_milliseconds*2/10)),
                [this]() {
                    second_laser_timer->cancel();
                    if (!use_sim_time) {
                        // Stay in LASERING so detection remains blocked.
                        // Run return spline in pose_callback; it will set NO_TARGET when done.
                        return_motion_elapsed_         = 0.0;
                        return_motion_last_tick_       = this->get_clock()->now();
                        return_motion_start_positions_ = joint_states;
                        return_motion_active_          = true;
                        RCLCPP_INFO(this->get_logger(), "Laser done. Starting return spline.");
                    } else {
                        controller_status = ControllerStatus::NO_TARGET;
                    }
                });
    }

    WeederControllerWithYolo::~WeederControllerWithYolo()
    {
    }

} // namespace arm mazzolini

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<arm_mazzolini::WeederControllerWithYolo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
