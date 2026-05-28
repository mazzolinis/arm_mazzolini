#include "arm_mazzolini/weeder_controller.hpp"

// This is the multi threaded version of weeder_controller that is uploaded on GitHub

namespace arm_mazzolini
{
    WeederController::WeederController() : Node("weeder_controller")
    {
        // ─────────────────────────────────────────────────────────────────
        // Callback groups — MUST be created BEFORE any subscription / timer
        // that uses them, otherwise the executor will not see the
        // association.
        // ─────────────────────────────────────────────────────────────────
        control_cb_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        perception_cb_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // Parameters
        declare_and_get_parameters();

        // Arm kinematic initialization
        arm_kinematic = std::make_unique<ArmKinematic>(l1, l2);

        // Detector strategy: build the desired-feature position once,
        // detector was already instantiated inside declare_and_get_parameters.
        desired_position = detector->PBTargetPosition(desired_feature);

        // SubscriptionOptions used to bind a subscription to a callback group.
        rclcpp::SubscriptionOptions perception_opts;
        perception_opts.callback_group = perception_cb_group_;
        rclcpp::SubscriptionOptions control_opts;
        control_opts.callback_group = control_cb_group_;

        if (!use_sim_time) {

            // ── Perception subscriptions (perception group) ──────────────
            auto qos = rclcpp::SensorDataQoS();
            rgb_sub_.subscribe(this, camera_rgb_topic, qos.get_rmw_qos_profile(), perception_opts);
            depth_sub_.subscribe(this, camera_depth_topic, qos.get_rmw_qos_profile(), perception_opts);
            real_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                camera_info_topic,
                rclcpp::SensorDataQoS(),
                std::bind(&WeederController::real_info_callback, this, std::placeholders::_1),
                perception_opts
            );
            real_sync = std::make_shared<message_filters::Synchronizer<RealSyncPolicy>>(RealSyncPolicy(10), rgb_sub_, depth_sub_);
            real_sync->setMaxIntervalDuration(rclcpp::Duration(0, 100000000));  // 100 ms
            real_sync->registerCallback(std::bind(&WeederController::real_image_callback, this, std::placeholders::_1, std::placeholders::_2));

            // ── Joint states subscription (control group) ────────────────
            states_subscription_ = this->create_subscription<pi3hat_moteus_int_msgs::msg::JointsStates>(
                real_joints_states_topic,
                10,
                std::bind(&WeederController::real_states_callback, this, std::placeholders::_1),
                control_opts
            );

            message_throttle_ms = message_throttle_ms * 5;

            // Command publisher
            command_publisher_ = this->create_publisher<pi3hat_moteus_int_msgs::msg::JointsCommand>(real_joints_command_topic, 10);
            // Laser publisher
            real_laser_pub = this->create_publisher<std_msgs::msg::Empty>("/shoot", 10);

        }
        else {

            // ── Perception subscriptions (perception group) ──────────────
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
            rgb_sub_.subscribe(this, camera_rgb_topic, qos.get_rmw_qos_profile(), perception_opts);
            depth_sub_.subscribe(this, camera_depth_topic, qos.get_rmw_qos_profile(), perception_opts);
            info_sub_.subscribe(this, camera_info_topic, qos.get_rmw_qos_profile(), perception_opts);
            sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), rgb_sub_, depth_sub_, info_sub_);
            sync_->registerCallback(std::bind(&WeederController::image_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

            // ── Joint states subscription (control group) ────────────────
            joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states",
                rclcpp::SensorDataQoS(),
                std::bind(&WeederController::joint_states_callback, this, std::placeholders::_1),
                control_opts
            );

            // Joint trajectory controller action client
            joints_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
                this,
                "/joint_trajectory_controller/follow_joint_trajectory",
                control_cb_group_   // action client also bound to the control group
            );
            if (joints_client->wait_for_action_server(std::chrono::seconds(10)) == false) {
                RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                RCLCPP_ERROR(this->get_logger(), "Shutting down...");
                rclcpp::shutdown();
            }
            goal_options.result_callback = std::bind(&WeederController::result_callback, this, std::placeholders::_1);
            goal_msg.trajectory.joint_names = joint_names;

            // Joint trajectory publisher (for POSITIONING / visual servoing)
            trajectory_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);

            // Laser publisher
            simulated_laser_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/lasered_position", 10);
        }

        // TF2 listener (no callback group: it has its own internal threads)
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        // Pose timer (control group): fires every tf_callback_period_ms.
        pose_timer = rclcpp::create_timer(
            this,
            this->get_clock(),
            rclcpp::Duration(std::chrono::milliseconds(tf_callback_period_ms)),
            std::bind(&WeederController::pose_timer_callback, this),
            control_cb_group_
        );
        last_warning_time = this->now();

        // Visual servoing timer (control group): fires every control_dt ms.
        control_timer = rclcpp::create_timer(
            this,
            this->get_clock(),
            rclcpp::Duration(std::chrono::milliseconds(control_dt)),
            std::bind(&WeederController::control_callback, this),
            control_cb_group_
        );
        last_detection_time = this->now();

        // Initial status
        new_pose.setIdentity();
        old_pose.setIdentity();
        controller_status = ControllerStatus::NO_TARGET;

        RCLCPP_DEBUG(this->get_logger(), "SETUP COMPLETE");
    }

    void WeederController::declare_and_get_parameters()
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
            this->declare_parameter("detector_type", std::string());
            this->declare_parameter("roi_size", int());
            this->declare_parameter("morph_kernel_size", int());
            this->declare_parameter("confidence_threshold", double());
            this->declare_parameter("depth_roi_size", int());
            this->declare_parameter("CUDA_device_id", int());
            this->declare_parameter("trajectory_time_ms", 250);
            this->declare_parameter("visual_servoing.filtering", true);
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

            // Detector selection
            detector_type_string = this->get_parameter("detector_type").as_string();
            auto it = detector_map.find(detector_type_string);
            if (it != detector_map.end()) {
                detector_type = it->second;
            } else {
                RCLCPP_WARN(this->get_logger(), "Invalid detector type parameter: %s", detector_type_string.c_str());
                RCLCPP_WARN(this->get_logger(), "Using default detector type: ExG_threshold");
                detector_type = DetectorType::ExG_threshold;
            }

            depth_roi_size = this->get_parameter("depth_roi_size").as_int();

            if (detector_type == DetectorType::YOLO) {
                confidence_threshold = this->get_parameter("confidence_threshold").as_double();
                CUDA_device_id = this->get_parameter("CUDA_device_id").as_int();
                detector = std::make_unique<WeedDetector>(model_path, confidence_threshold, depth_roi_size, CUDA_device_id);
            }
            else {
                roi_size = this->get_parameter("roi_size").as_int();
                morph_kernel_size = this->get_parameter("morph_kernel_size").as_int();
                detector = std::make_unique<SphereDetector>(roi_size, morph_kernel_size, depth_roi_size, detector_type);
            }

            // Visual servoing parameters
            filtering = this->get_parameter("visual_servoing.filtering").as_bool();
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
            return;   // make sure we do not keep executing after shutdown
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Image callback (simulation, perception group)
    // ─────────────────────────────────────────────────────────────────────
    void WeederController::image_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr depth_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg)
    {
        if (!camera_initialized) {
            // camera info are assumed constant across the session
            detector->SetCameraInfo(info_msg);
            camera_initialized = true;
            RCLCPP_DEBUG(this->get_logger(), "Camera initialized");
        }

        // Snapshot of controller_status under lock so we react to a
        // coherent value even if the control group is mutating it.
        ControllerStatus status_snap;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            status_snap = controller_status;
        }
        if (status_snap == ControllerStatus::LASERING) {
            return;
        }

        // Time-based throttle (perception-local member, no lock needed).
        auto now = this->now();
        double elapsed_ms = (now - last_detection_time).seconds() * 1000.0;
        if (elapsed_ms < detection_period_ms)
            return;
        last_detection_time = now;

        switch (status_snap) {
            case ControllerStatus::NO_TARGET:
            {
                Eigen::Vector3d actual_position;
                RCLCPP_DEBUG(this->get_logger(), "Detecting target...");

                if (!detector->PBTargetPosition(rgb_msg, depth_msg, actual_position)) {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *(this->get_clock()), message_throttle_ms, "No target detected");
                    return;
                }
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    controller_status = ControllerStatus::HAS_TARGET;
                }
                return_motion_active_ = false;
                [[fallthrough]];
            }

            case ControllerStatus::HAS_TARGET:
            {
                Eigen::Vector3d actual_position;

                if (!detector->PBTargetPosition(rgb_msg, depth_msg, actual_position)) {
                    RCLCPP_INFO(this->get_logger(), "Target lost!");
                    target_buffer.clear();
                    {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        controller_status = ControllerStatus::NO_TARGET;
                    }
                    return;
                }

                target_buffer.push_back(actual_position);
                RCLCPP_DEBUG(this->get_logger(), "Target detected at [%.3f, %.3f, %.3f] in camera frame",
                    actual_position[0], actual_position[1], actual_position[2]);

                if (target_buffer.size() >= image_buffer_size) {
                    // Buffer full: compute average position and ask the
                    // control group to start moving the arm.
                    Eigen::Vector3d avg_position = Eigen::Vector3d::Zero();
                    for (const auto& pos : target_buffer) {
                        avg_position += pos;
                    }
                    avg_position /= static_cast<double>(image_buffer_size);
                    target_buffer.clear();
                    RCLCPP_INFO(this->get_logger(), "Target found. STOP!");

                    try {
                        geometry_msgs::msg::TransformStamped camera_pose =
                            tf_buffer->lookupTransform("kinematic_link", "camera_kinematic", tf2::TimePointZero);
                        Eigen::Isometry3d camera_to_kinematic = tf2::transformToEigen(camera_pose);
                        Eigen::Vector3d position_in_kinematic = camera_to_kinematic * avg_position;

                        // Publish the new target to the control group.
                        {
                            std::lock_guard<std::mutex> lock(state_mutex_);
                            target_camera_position = avg_position;
                            target_position = position_in_kinematic;
                            controller_status = ControllerStatus::ARM_MOVING;
                        }

                        // Debug print only (read-only TF lookups, no shared state).
                        geometry_msgs::msg::TransformStamped base_to_kinematic =
                            tf_buffer->lookupTransform("kinematic_link", "arm_base_link", tf2::TimePointZero);
                        Eigen::Isometry3d base_to_kinematic_eigen = tf2::transformToEigen(base_to_kinematic);
                        geometry_msgs::msg::TransformStamped base_to_camera =
                            tf_buffer->lookupTransform("arm_base_link", "camera_kinematic", tf2::TimePointZero);
                        Eigen::Isometry3d base_to_camera_eigen = tf2::transformToEigen(base_to_camera);
                        Eigen::Vector3d target_in_arm_base = base_to_kinematic_eigen * position_in_kinematic;

                        // Joint states snapshot taken just for the debug line.
                        std::vector<double> joint_states_local;
                        {
                            std::lock_guard<std::mutex> lock(state_mutex_);
                            joint_states_local = joint_states;
                        }

                        RCLCPP_DEBUG_STREAM(this->get_logger(), "====================== DEBUG INVERSE KINEMATIC: ======================");
                        RCLCPP_DEBUG_STREAM(this->get_logger(), "Joint states: " << joint_states_local[0] << ", " << joint_states_local[1]);
                        RCLCPP_DEBUG_STREAM(this->get_logger(), "Target in camera frame: " << std::endl << avg_position);
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
                break;
            }

            case ControllerStatus::ARM_MOVING:
            {
                break;
            }

            case ControllerStatus::POSITIONING:
            {
                Eigen::Vector3d feature_local;
                if (!detector->IBTargetPosition(rgb_msg, depth_msg, feature_local)) {
                    RCLCPP_INFO(this->get_logger(), "Target lost!");
                    target_buffer.clear();
                    {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        controller_status = ControllerStatus::NO_TARGET;
                    }
                    return;
                }

                Eigen::Vector3d position_local = detector->PBTargetPosition(feature_local);
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    target_feature = feature_local;
                    target_camera_position = position_local;
                }
                RCLCPP_INFO_THROTTLE(this->get_logger(), *(this->get_clock()), message_throttle_ms,
                    "FOLLOWING TARGET AT POSITION [%.3f, %.3f, %.3f] IN CAMERA FRAME",
                    position_local[0], position_local[1], position_local[2]
                );
                break;
            }

            case ControllerStatus::LASERING:
            {
                // already filtered above, kept for switch completeness
                break;
            }
        }
    }

    void WeederController::real_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg)
    {
        if (!camera_initialized) {
            detector->SetCameraInfo(info_msg);
            camera_initialized = true;
            RCLCPP_INFO(this->get_logger(), "Camera info received, initialized Detector");
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Image callback (real hardware, perception group)
    // ─────────────────────────────────────────────────────────────────────
    void WeederController::real_image_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr depth_msg)
    {
        if (!camera_initialized) {
            RCLCPP_WARN(this->get_logger(), "Camera info not received yet, cannot initialize Detector");
            return;
        }

        ControllerStatus status_snap;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            status_snap = controller_status;
        }
        if (status_snap == ControllerStatus::LASERING) {
            return;
        }

        auto now = this->now();
        double elapsed_ms = (now - last_detection_time).seconds() * 1000.0;
        if (elapsed_ms < detection_period_ms)
            return;
        last_detection_time = now;

        switch (status_snap) {
            case ControllerStatus::NO_TARGET:
            {
                Eigen::Vector3d actual_position;
                RCLCPP_DEBUG(this->get_logger(), "Detecting target...");

                if (!detector->PBTargetPosition(rgb_msg, depth_msg, actual_position)) {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *(this->get_clock()), message_throttle_ms, "No target detected");
                    return;
                }
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    controller_status = ControllerStatus::HAS_TARGET;
                }
                return_motion_active_ = false;
                [[fallthrough]];
            }

            case ControllerStatus::HAS_TARGET:
            {
                Eigen::Vector3d actual_position;

                if (!detector->PBTargetPosition(rgb_msg, depth_msg, actual_position)) {
                    RCLCPP_INFO(this->get_logger(), "Target lost!");
                    target_buffer.clear();
                    {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        controller_status = ControllerStatus::NO_TARGET;
                    }
                    return;
                }

                target_buffer.push_back(actual_position);
                RCLCPP_DEBUG(this->get_logger(), "Target detected at [%.3f, %.3f, %.3f] in camera frame",
                    actual_position[0], actual_position[1], actual_position[2]);

                if (target_buffer.size() >= image_buffer_size) {
                    Eigen::Vector3d avg_position = Eigen::Vector3d::Zero();
                    for (const auto& pos : target_buffer) {
                        avg_position += pos;
                    }
                    avg_position /= static_cast<double>(image_buffer_size);
                    target_buffer.clear();

                    try {
                        geometry_msgs::msg::TransformStamped camera_pose =
                            tf_buffer->lookupTransform("kinematic_link", "camera_kinematic", tf2::TimePointZero);
                        Eigen::Isometry3d camera_to_kinematic = tf2::transformToEigen(camera_pose);
                        Eigen::Vector3d position_in_kinematic = camera_to_kinematic * avg_position;

                        {
                            std::lock_guard<std::mutex> lock(state_mutex_);
                            target_camera_position = avg_position;
                            target_position = position_in_kinematic;
                            controller_status = ControllerStatus::ARM_MOVING;
                        }
                        RCLCPP_DEBUG(this->get_logger(), "Target locked.");
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
                break;
            }

            case ControllerStatus::ARM_MOVING:
            {
                break;
            }

            case ControllerStatus::POSITIONING:
            {
                Eigen::Vector3d feature_local;
                if (!detector->IBTargetPosition(rgb_msg, depth_msg, feature_local)) {
                    RCLCPP_INFO(this->get_logger(), "Target lost!");
                    target_buffer.clear();
                    {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        controller_status = ControllerStatus::NO_TARGET;
                    }
                    return;
                }

                Eigen::Vector3d position_local = detector->PBTargetPosition(feature_local);
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    target_feature = feature_local;
                    target_camera_position = position_local;
                }
                RCLCPP_INFO_THROTTLE(this->get_logger(), *(this->get_clock()), message_throttle_ms,
                    "FOLLOWING TARGET AT POSITION [%.3f, %.3f, %.3f] IN CAMERA FRAME",
                    position_local[0], position_local[1], position_local[2]
                );
                break;
            }

            case ControllerStatus::LASERING:
            {
                break;
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Joint state callbacks (control group)
    // ─────────────────────────────────────────────────────────────────────
    void WeederController::joint_states_callback(
        const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Read controller_status under lock to coordinate with perception.
        ControllerStatus status_snap;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            status_snap = controller_status;
        }
        if (status_snap == ControllerStatus::NO_TARGET ||
            status_snap == ControllerStatus::LASERING) {
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

        if (idx1 < 0 || idx2 < 0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "joint1 or joint2 not found in /joint_states"
            );
            return;
        }

        double j0 = msg->position[idx1];
        double j1 = msg->position[idx2];
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            joint_states[0] = j0;
            joint_states[1] = j1;
        }

        // RCLCPP_DEBUG(this->get_logger(), "Joint states updated: joint1=%.3f joint2=%.3f", j0, j1);
    }

    void WeederController::real_states_callback(
        const pi3hat_moteus_int_msgs::msg::JointsStates::SharedPtr msg)
    {
        // Read controller_status under lock.
        ControllerStatus status_snap;
        std::vector<double> joint_states_snap;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            status_snap = controller_status;
            joint_states_snap = joint_states;   // start from current state
        }
        if (status_snap == ControllerStatus::LASERING) {
            return;
        }

        // Work on a local copy: if something is wrong with the message,
        // we do not want to overwrite the previous good joint state.
        std::vector<double> joint_positions = joint_states_snap;
        for (size_t i = 0; i < msg->name.size(); i++) {
            if (std::isfinite(msg->position[i])) {
                size_t idx = std::find(joint_names.begin(), joint_names.end(), msg->name[i]) - joint_names.begin();
                if (idx < joint_names.size()) {
                    joint_positions[idx] = msg->position[i];
                }
            }
            else if (std::isfinite(msg->sec_enc_pos[i])) {
                size_t idx = std::find(joint_names.begin(), joint_names.end(), msg->name[i]) - joint_names.begin();
                if (idx < joint_names.size()) {
                    joint_positions[idx] = msg->sec_enc_pos[i];
                }
            }
            else {
                continue;
            }
        }

        // Commit updated values under lock.
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            joint_states = joint_positions;
            joint_states_received_ = true;
        }

        RCLCPP_DEBUG(this->get_logger(), "Real Joint states updated: joint1=%.3f joint2=%.3f",
            joint_positions[0], joint_positions[1]);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Pose timer / pose callback (control group)
    // ─────────────────────────────────────────────────────────────────────
    void WeederController::pose_timer_callback()
    {
        if (use_sim_time) {

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
            // Force the pose to be identity on the real robot for now.
            // TODO: read the real mobile-base pose.
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

    void WeederController::pose_callback(geometry_msgs::msg::TransformStamped msg)
    {
        new_pose = tf2::transformToEigen(msg);

        // Snapshot of shared state under lock. We need both controller_status
        // and joint_states for the various branches; copying them once at
        // the top keeps the critical section short.
        ControllerStatus status_snap;
        std::vector<double> joint_states_snap;
        Eigen::Vector3d target_position_snap;
        bool joint_states_received_snap;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            status_snap                 = controller_status;
            joint_states_snap           = joint_states;
            target_position_snap        = target_position;
            joint_states_received_snap  = joint_states_received_;
        }

        switch (status_snap) {

            case ControllerStatus::NO_TARGET:
            {
                if (use_sim_time) {
                    send_joint_trajectory(initial_joint_values);
                }
                else {
                    // Real hardware: smooth spline back to initial_joint_values.
                    if (!joint_states_received_snap) {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *(this->get_clock()),
                            message_throttle_ms, "Waiting for real joint states before return motion...");
                        break;
                    }

                    if (!return_motion_active_) {
                        if (vectors_are_equal(joint_states_snap, initial_joint_values)) {
                            return;
                        }
                        return_motion_elapsed_         = 0.0;
                        return_motion_last_tick_       = this->get_clock()->now();
                        return_motion_start_positions_ = joint_states_snap;
                        return_motion_active_          = true;
                        RCLCPP_DEBUG(this->get_logger(), "======================================================================");
                        RCLCPP_DEBUG(this->get_logger(),
                            "NEW SPLINE START: [%.3f, %.3f] -> [%.3f, %.3f] in %.1f s",
                            joint_states_snap[0], joint_states_snap[1],
                            initial_joint_values[0], initial_joint_values[1],
                            return_motion_duration);
                        RCLCPP_DEBUG(this->get_logger(), "======================================================================");
                    } else {
                        auto now = this->get_clock()->now();
                        const double max_delta = static_cast<double>(tf_callback_period_ms) / 1000.0;
                        double delta = std::min((now - return_motion_last_tick_).seconds(), max_delta);
                        return_motion_elapsed_ += delta;
                        return_motion_last_tick_ = now;
                    }

                    double t = std::min(return_motion_elapsed_ / return_motion_duration, 1.0);
                    double s = t * t * (3.0 - 2.0 * t);
                    std::vector<double> interp(initial_joint_values.size());
                    for (size_t i = 0; i < interp.size(); i++) {
                        interp[i] = return_motion_start_positions_[i] * (1.0 - s)
                                  + initial_joint_values[i] * s;
                    }

                    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "SPLINE DEBUG: " << std::endl
                        << "Return motion: elapsed=" << return_motion_elapsed_ << "s t=" << t
                        << " current=[" << joint_states_snap[0] << ", " << joint_states_snap[1] << "]"
                        << " interp=[" << interp[0] << ", " << interp[1] << "]"
                        << " target=[" << initial_joint_values[0] << ", " << initial_joint_values[1] << "]"
                    );

                    send_joint_trajectory(interp);

                    if (t >= 1.0) {
                        return_motion_active_ = false;
                        RCLCPP_INFO(this->get_logger(), "INITIAL POSITION REACHED");
                        send_joint_trajectory(initial_joint_values);
                    }
                }
                break;
            }

            case ControllerStatus::HAS_TARGET:
            {
                if (!new_pose.isApprox(old_pose, pose_threshold)) {
                    RCLCPP_INFO(this->get_logger(), "STOP!");
                    target_buffer.clear();
                }
                break;
            }

            case ControllerStatus::ARM_MOVING:
            {
                if (!new_pose.isApprox(old_pose, pose_threshold)) {
                    RCLCPP_INFO(this->get_logger(), "STOP!! Arm moving. Movement aborted.");
                    target_buffer.clear();
                    {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        controller_status = ControllerStatus::HAS_TARGET;
                    }
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
                    if (arm_kinematic->computeIK(target_position_snap, joint_angles, error_type)) {
                        if (use_sim_time) {
                            RCLCPP_DEBUG_STREAM(this->get_logger(), "Arm moving to: " << joint_angles[0] << ", " << joint_angles[1]);
                            send_joint_trajectory(joint_angles);
                        }
                        else {
                            if (!arm_motion_active_) {
                                arm_motion_start_time_       = this->get_clock()->now();
                                arm_motion_start_positions_  = joint_states_snap;
                                arm_motion_target_positions_ = joint_angles;
                                arm_motion_active_           = true;
                                RCLCPP_INFO(this->get_logger(),
                                    "Starting arm spline: [%.3f, %.3f] -> [%.3f, %.3f] in %.1f s",
                                    joint_states_snap[0], joint_states_snap[1],
                                    joint_angles[0], joint_angles[1],
                                    forward_motion_duration);
                            }

                            double elapsed = (this->get_clock()->now() - arm_motion_start_time_).seconds();
                            double t = std::min(elapsed / forward_motion_duration, 1.0);
                            double s = t * t * (3.0 - 2.0 * t);
                            std::vector<double> interp(arm_motion_target_positions_.size());
                            for (size_t i = 0; i < interp.size(); i++) {
                                interp[i] = arm_motion_start_positions_[i] * (1.0 - s)
                                          + arm_motion_target_positions_[i] * s;
                            }

                            send_joint_trajectory(interp);

                            if (t >= 1.0) {
                                arm_motion_active_ = false;
                                {
                                    std::lock_guard<std::mutex> lock(state_mutex_);
                                    controller_status = ControllerStatus::POSITIONING;
                                }
                                RCLCPP_INFO(this->get_logger(), "Arm reached target, switching to POSITIONING");
                            }
                        }
                    }
                    else {
                        switch (error_type) {
                            case ErrorType::TARGET_EMPTY:
                                RCLCPP_WARN(this->get_logger(), "Sent empty joint angles, check for error");
                                controller_status = ControllerStatus::NO_TARGET; // go back to initial position
                                return;
                            case ErrorType::TARGET_TOO_FAR:
                            {
                                RCLCPP_INFO_THROTTLE(this->get_logger(), *(this->get_clock()), message_throttle_ms, "Get closer, target out of reach.");
                                double r = std::sqrt(target_position_snap.x()*target_position_snap.x() + target_position_snap.y()*target_position_snap.y());
                                RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *(this->get_clock()), message_throttle_ms, "r = " << r << " l1 + l2 = " << (l1 + l2));
                                controller_status = ControllerStatus::NO_TARGET; // go back to initial position
                                return;
                            }
                            case ErrorType::EXCLUSION_ZONE:
                                RCLCPP_INFO_THROTTLE(this->get_logger(), *(this->get_clock()), message_throttle_ms, "Target unreachable, move around obstacles");
                                controller_status = ControllerStatus::NO_TARGET; // go back to initial position
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
                    {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        target_feature.setZero();
                        target_camera_position.setZero();
                        controller_status = ControllerStatus::NO_TARGET;
                    }
                    if (use_sim_time) {
                        joints_client->async_cancel_all_goals();
                    }
                }
                break;
            }

            case ControllerStatus::LASERING:
            {
                // LASERING is the ONLY state in which pose_callback is allowed
                // to publish joint commands: it runs the return spline while
                // perception remains blocked (see image_callback / control_callback).
                if (!use_sim_time && return_motion_active_) {
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
                        {
                            std::lock_guard<std::mutex> lock(state_mutex_);
                            controller_status = ControllerStatus::NO_TARGET;
                        }
                        RCLCPP_INFO(this->get_logger(), "RETURN SPLINE COMPLETE. Ready for new target.");
                    }
                } else if (!new_pose.isApprox(old_pose, pose_threshold)) {
                    {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        controller_status = ControllerStatus::NO_TARGET;
                    }
                    RCLCPP_INFO(this->get_logger(), "Mobile robot moved during LASERING");
                }
                break;
            }

        }

        old_pose = new_pose;
    }

    // ─────────────────────────────────────────────────────────────────────
    // send_joint_trajectory: re-checks LASERING under lock as a last-line
    // safety net. The robot must remain still during LASERING, so even if a
    // caller somehow reaches here in LASERING the command is dropped.
    // ─────────────────────────────────────────────────────────────────────
    void WeederController::send_joint_trajectory(const std::vector<double>& joint_angles)
    {
        if (joint_angles.size() != joint_names.size()) {
            RCLCPP_ERROR(this->get_logger(), "Joint angles size does not match joint names size");
            return;
        }
        if (vectors_are_equal(joint_angles, last_joint_angles)) {
            RCLCPP_DEBUG(this->get_logger(), "Joint angles are the same as last sent, not sending trajectory");
            return;
        }

        // Safety net: never publish during LASERING unless we are running
        // the return spline (which sets return_motion_active_ in pose_callback).
        ControllerStatus status_snap;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            status_snap = controller_status;
        }
        if (status_snap == ControllerStatus::LASERING && !return_motion_active_) {
            RCLCPP_DEBUG(this->get_logger(), "Suppressing command publish during LASERING");
            return;
        }

        last_joint_angles = joint_angles;

        if (use_sim_time) {
            if (status_snap == ControllerStatus::POSITIONING) {
                // Visual servoing: use the trajectory publisher, not the action.
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
            command_msg.velocity = std::vector<double>(joint_angles.size(), 0.0);
            command_msg.effort   = std::vector<double>(joint_angles.size(), 0.0);
            command_msg.kp_scale = kp_scale;
            command_msg.kd_scale = kd_scale;
            command_publisher_->publish(command_msg);
        }
    }

    void WeederController::result_callback(const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result)
    {
        ControllerStatus status_snap;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            status_snap = controller_status;
        }
        if (status_snap == ControllerStatus::LASERING) {
            return;
        }

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
            {
                if (status_snap == ControllerStatus::ARM_MOVING) {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    controller_status = ControllerStatus::POSITIONING;
                }
                break;
            }
            case rclcpp_action::ResultCode::ABORTED:
                break;
            case rclcpp_action::ResultCode::CANCELED:
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
                break;
        }
    }

    bool WeederController::vectors_are_equal(const std::vector<double>& vec1, const std::vector<double>& vec2)
    {
        if (vec1.size() != vec2.size()) {
            return false;
        }
        for (size_t i = 0; i < vec1.size(); i++) {
            if (std::abs(vec1[i] - vec2[i]) > joint_tolerance) {
                return false;
            }
            else if (!use_sim_time && std::abs(vec1[i] - vec2[i]) > joint_tolerance * 1.0) {
                return false;
            }
        }
        return true;
    }

    // ─────────────────────────────────────────────────────────────────────
    // control_callback: the IBVS loop. Runs in the control group, so it is
    // serialized with pose_timer_callback. Reads shared state under lock,
    // performs heavy math on local copies, then publishes.
    // ─────────────────────────────────────────────────────────────────────
    void WeederController::control_callback()
    {
        // Snapshot all the shared state we will need.
        ControllerStatus status_snap;
        Eigen::Vector3d  feature_local;
        std::vector<double> joint_states_local;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            status_snap        = controller_status;
            feature_local      = target_feature;
            joint_states_local = joint_states;
        }

        // Hard stop: no command issued while LASERING.
        if (status_snap == ControllerStatus::LASERING) {
            return;
        }
        if (status_snap != ControllerStatus::POSITIONING) {
            return;
        }
        if (feature_local.isZero()) {
            RCLCPP_DEBUG(this->get_logger(), "Target feature is zero, skipping control");
            return;
        }

        double pixel_error = std::sqrt(
            std::pow(feature_local.x() - desired_feature.x(), 2) +
            std::pow(feature_local.y() - desired_feature.y(), 2));

        // --- DIAGNOSTIC (temporary) ---
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *(this->get_clock()), 1000,
            "CTRL: feature=[%.4f, %.4f, %.4f] desired=[%.4f, %.4f] pixel_error=%.4f threshold=%.4f",
            feature_local.x(), feature_local.y(), feature_local.z(),
            desired_feature.x(), desired_feature.y(), pixel_error, control_threshold);
        // --- END DIAGNOSTIC ---


        if (pixel_error < control_threshold) {
            activate_laser();
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                target_feature.setZero();
            }
        }
        else {
            std::vector<double> joint_goal = IBVS_control(feature_local, joint_states_local);
            send_joint_trajectory(joint_goal);
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // IBVS_control now takes a snapshot of joint_states so the caller is
    // in charge of locking. No shared-state reads inside this function.
    // ─────────────────────────────────────────────────────────────────────
    std::vector<double> WeederController::IBVS_control(
        Eigen::Vector3d& feature,
        const std::vector<double>& joint_states_snapshot)
    {
        double u = feature.x();
        double v = feature.y();
        double Z = feature.z();

        double dt = static_cast<double>(control_dt) / 1000.0;

        Eigen::Vector3d e_s;
        if (filtering) {
            e_s = feature - desired_feature;
            // Smoothing error
            e_s_filtered = beta.cwiseProduct(e_s) + (Eigen::Vector3d(1.0, 1.0, 1.0) - beta).cwiseProduct(e_s_filtered);
            e_s = e_s_filtered;
        }
        else {
            e_s = feature - desired_feature;
        }

        // Direct pseudo-inverse of the simplified interaction matrix
        // (depth camera + translation only).
        Eigen::Matrix3d L_pinv;
        L_pinv <<  -Z,    0.0,   u,
                   0.0,   -Z,    v,
                   0.0,   0.0,  -1.0;

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

        // Camera -> EE rotation (constant, read from TF).
        Eigen::Isometry3d camera_to_EE;
        try {
            geometry_msgs::msg::TransformStamped camera_pose =
                tf_buffer->lookupTransform("end_effector", "camera_kinematic", tf2::TimePointZero);
            camera_to_EE = tf2::transformToEigen(camera_pose);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *(this->get_clock()), message_throttle_ms,
                "Could not transform camera_kinematic to kinematic_link: %s", ex.what());
            return {joint_states_snapshot[0], joint_states_snapshot[1]};
        }
        Eigen::Matrix3d R_ec = camera_to_EE.rotation();

        Eigen::Vector3d v_e = R_ec * v_c;

        // SCARA Jacobian (uses the snapshot, no shared-state access).
        Eigen::Matrix3d J = arm_kinematic->computeScaraJacobian(joint_states_snapshot);

        Eigen::Vector3d q_dot;
        if (use_sim_time) {
            q_dot = J.inverse() * v_e;
        }
        else {
            // Singularity guard: damped least-squares inversion.
            const double mu = 0.05;
            Eigen::Matrix3d J_damped = J.transpose() * (J * J.transpose() + mu * mu * Eigen::Matrix3d::Identity()).inverse();
            q_dot = J_damped * v_e;
        }

        auto q = Eigen::Vector3d(joint_states_snapshot[0], joint_states_snapshot[1], 0.0);
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

    // ─────────────────────────────────────────────────────────────────────
    // activate_laser: enter LASERING and arm the two one-shot laser timers.
    // The timers are explicitly bound to control_cb_group_ so they are
    // serialized with pose_timer / control_callback.
    // ─────────────────────────────────────────────────────────────────────
    void WeederController::activate_laser()
    {
        // Atomic transition into LASERING.
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            controller_status = ControllerStatus::LASERING;
        }

        int pause_milliseconds;
        if (use_sim_time) {
            pause_milliseconds = 500;
        }
        else {
            pause_milliseconds = 10000;
        }

        RCLCPP_INFO(this->get_logger(), "============ ACTIVATING LASER =============");

        if (!use_sim_time) {
            real_laser_pub->publish(std_msgs::msg::Empty());
        }

        first_laser_timer = rclcpp::create_timer(
            this,
            this->get_clock(),
            rclcpp::Duration(std::chrono::milliseconds(pause_milliseconds * 8 / 10)),
            [this]() {
                first_laser_timer->cancel();

                geometry_msgs::msg::PointStamped lasered_position;
                if (use_sim_time) {
                    try {
                        // Snapshot target_position under lock for the publish.
                        Eigen::Vector3d target_position_local;
                        {
                            std::lock_guard<std::mutex> lock(state_mutex_);
                            target_position_local = target_position;
                        }
                        // TODO: remove odom and use a global fixed frame.
                        geometry_msgs::msg::TransformStamped arm_pose =
                            tf_buffer->lookupTransform("odom", "kinematic_link", tf2::TimePointZero);
                        Eigen::Isometry3d kinematic_to_base = tf2::transformToEigen(arm_pose);
                        Eigen::Vector3d absolute_position = kinematic_to_base * target_position_local;
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
            },
            control_cb_group_   // bind to control group
        );

        RCLCPP_INFO(this->get_logger(), "TARGET CLEARED. READY FOR A NEW ONE");

        // Second timer: avoid detecting the same plant twice. On real
        // hardware it kicks off the return spline (run by pose_callback);
        // in simulation it just exits LASERING.
        second_laser_timer = rclcpp::create_timer(
            this,
            this->get_clock(),
            rclcpp::Duration(std::chrono::milliseconds(pause_milliseconds * 2 / 10)),
            [this]() {
                second_laser_timer->cancel();
                if (!use_sim_time) {
                    // Stay in LASERING so detection remains blocked.
                    // pose_callback runs the return spline and will set
                    // NO_TARGET when it is done.
                    std::vector<double> joint_states_local;
                    {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        joint_states_local = joint_states;
                    }
                    return_motion_elapsed_         = 0.0;
                    return_motion_last_tick_       = this->get_clock()->now();
                    return_motion_start_positions_ = joint_states_local;
                    return_motion_active_          = true;
                    RCLCPP_INFO(this->get_logger(), "Laser done. Starting return spline.");
                } else {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    controller_status = ControllerStatus::NO_TARGET;
                }
            },
            control_cb_group_   // bind to control group
        );
    }

    WeederController::~WeederController()
    {
    }

} // namespace arm_mazzolini

// ─────────────────────────────────────────────────────────────────────────
// main(): MultiThreadedExecutor with 2 threads, one per callback group.
// ─────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<arm_mazzolini::WeederController>();

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(),
        2 /* number of threads: one for control, one for perception */
    );
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
