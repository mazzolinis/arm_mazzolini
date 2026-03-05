#include "arm_mazzolini/weeder_controller.hpp"

namespace arm_mazzolini
{
    WeederController::WeederController() : Node("weeder_controller")
    {
        // Parameters
        declare_and_get_parameters();

        // Arm kinematic initialization
        arm_kinematic = std::make_unique<ArmKinematic>(l1, l2);

        // Image detector initialization
        sphere_detector = std::make_unique<SphereDetector>(roi_size, morph_kernel_size, depth_roi_size, mask_type);
        desired_position = sphere_detector->PBTargetPosition(desired_feature);

        // Image subscriptions
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        rgb_sub_.subscribe(this, camera_rgb_topic, qos.get_rmw_qos_profile());
        depth_sub_.subscribe(this, camera_depth_topic, qos.get_rmw_qos_profile());
        info_sub_.subscribe(this, camera_info_topic, qos.get_rmw_qos_profile());
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), rgb_sub_, depth_sub_, info_sub_);
        sync_->registerCallback(std::bind(&WeederController::image_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        // Joint states subscription
        joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 
            rclcpp::SensorDataQoS(),
            std::bind(&WeederController::joint_states_callback, this, std::placeholders::_1)
        );

        // TF2 listener
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        pose_timer = rclcpp::create_timer(
            this,
            this->get_clock(),
            rclcpp::Duration(std::chrono::milliseconds(tf_callback_period_ms)),
            std::bind(&WeederController::pose_timer_callback, this)
        );
        last_warning_time = this->now();

        // Action Client
        joints_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            this,
            "/joint_trajectory_controller/follow_joint_trajectory"
        );
        if (joints_client->wait_for_action_server(std::chrono::seconds(10)) == false) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            RCLCPP_ERROR(this->get_logger(), "Shutting down...");
            rclcpp::shutdown();
        }
        goal_options.result_callback = std::bind(&WeederController::result_callback, this, std::placeholders::_1);
        goal_msg.trajectory.joint_names = joint_names;

        // Publisher
        laser_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/lasered_position", 10);

        // Visual servoing
        control_timer = rclcpp::create_timer(
            this,
            this->get_clock(),
            rclcpp::Duration(std::chrono::milliseconds(control_dt)),
            std::bind(&WeederController::control_callback, this)
        );

        // Initial status
        new_pose.setIdentity();
        old_pose.setIdentity();
        controller_status = ControllerStatus::NO_TARGET;
    }

    void WeederController::declare_and_get_parameters()
    {
        try {
            // Declarations
            this->declare_parameter("link1_length", double());
            this->declare_parameter("link2_length", double());  
            this->declare_parameter("tf_callback_period", int());
            this->declare_parameter("image_buffer_size", int());
            this->declare_parameter("frames_delay", int());
            this->declare_parameter("camera_rgb_topic", std::string());
            this->declare_parameter("camera_depth_topic", std::string());
            this->declare_parameter("camera_info_topic", std::string());
            this->declare_parameter("roi_size", int());
            this->declare_parameter("morph_kernel_size", int());
            this->declare_parameter("depth_roi_size", int());    
            this->declare_parameter("trajectory_time_ms", 250);
            this->declare_parameter("mask_type", std::string());
            this->declare_parameter("visual_servoing.filtering", true);
            // this->declare_parameter("visual_servoing.alpha", std::vector<double>());
            this->declare_parameter("visual_servoing.beta", std::vector<double>());
            this->declare_parameter("visual_servoing.lambda_IBVS", std::vector<double>());
            this->declare_parameter("visual_servoing.lambda_PBVS", std::vector<double>());
            this->declare_parameter("visual_servoing.controller_period_ms", int());
            this->declare_parameter("visual_servoing.smoothing", double());
            this->declare_parameter("visual_servoing.control_threshold", 0.005);
            this->declare_parameter("visual_servoing.desired_feature", std::vector<double>());

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
            frames_delay = this->get_parameter("frames_delay").as_int();
            trajectory_time_ms = this->get_parameter("trajectory_time_ms").as_int();

            // Camera parameters
            roi_size = this->get_parameter("roi_size").as_int();
            morph_kernel_size = this->get_parameter("morph_kernel_size").as_int();
            depth_roi_size = this->get_parameter("depth_roi_size").as_int();
            std::string mask_type_str = this->get_parameter("mask_type").as_string();
            if(mask_type_str == "HSV_red") {
                mask_type = MaskType::HSV_red;
            }
            else if(mask_type_str == "ExG_threshold") {
                mask_type = MaskType::ExG_threshold;
            }
            else if(mask_type_str == "ExG_Otsu") {
                mask_type = MaskType::ExG_Otsu;
            }
            else if(mask_type_str == "ExGR") {
                mask_type = MaskType::ExGR;
            }
            else {
                RCLCPP_WARN(this->get_logger(), "Invalid mask type parameter: %s", mask_type_str.c_str());
                RCLCPP_WARN(this->get_logger(), "Using default mask type: HSV_red");
                mask_type = MaskType::HSV_red;
            }

            // Visual servoing parameters
            filtering = this->get_parameter("visual_servoing.filtering").as_bool();
            // alpha = Eigen::Vector3d::Map(this->get_parameter("visual_servoing.alpha").as_double_array().data());
            beta = Eigen::Vector3d::Map(this->get_parameter("visual_servoing.beta").as_double_array().data());

            std::vector<double> lambda_IBVS_vec = this->get_parameter("visual_servoing.lambda_IBVS").as_double_array();
            lambda_IBVS.diagonal() << lambda_IBVS_vec[0], lambda_IBVS_vec[1], lambda_IBVS_vec[2];
            std::vector<double> lambda_PBVS_vec = this->get_parameter("visual_servoing.lambda_PBVS").as_double_array();
            lambda_PBVS.diagonal() << lambda_PBVS_vec[0], lambda_PBVS_vec[1], lambda_PBVS_vec[2];

            control_dt = this->get_parameter("visual_servoing.controller_period_ms").as_int();
            double smoothing = this->get_parameter("visual_servoing.smoothing").as_double();
            trajectory_dt = static_cast<int>(control_dt * smoothing);
            control_threshold = this->get_parameter("visual_servoing.control_threshold").as_double();
            desired_feature = Eigen::Vector3d::Map(this->get_parameter("visual_servoing.desired_feature").as_double_array().data());
            
        } 

        catch(const rclcpp::exceptions::InvalidParameterTypeException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Error in params declaration: %s", ex.what());
            RCLCPP_ERROR(this->get_logger(), "Shutting down...");
            rclcpp::shutdown();
        }
    }

    void WeederController::image_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr depth_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg)
    {
        if (!camera_initialized) {
            // Hypothesis: camera info are constant
            sphere_detector->SetCameraInfo(info_msg);
            camera_initialized = true;
        }

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
                        
                        if(!sphere_detector->PBTargetPosition(rgb_msg, depth_msg, actual_position)) {
                            RCLCPP_INFO_THROTTLE(this->get_logger(), *(this->get_clock()), message_throttle_ms, "No target detected");
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
                    if(!sphere_detector->PBTargetPosition(rgb_msg, depth_msg, actual_position)) {
                        RCLCPP_INFO(this->get_logger(), "Target lost!");
                        target_buffer.clear();
                        controller_status = ControllerStatus::NO_TARGET;
                        return;
                    }
                    else {
                        target_buffer.push_back(actual_position);
                        // RCLCPP_INFO(this->get_logger(), "Target detected at [%.3f, %.3f, %.3f] in camera frame", actual_position.x(), actual_position.y(), actual_position.z());
                        
                        if(target_buffer.size() >= image_buffer_size) {
                            // Compute temporal average and send trajectory
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
                                // store target position and use it later, DO NOT MOVE NOW!

                                // Debug print:
                                RCLCPP_DEBUG_STREAM(this->get_logger(), "Target obtained: ");
                                RCLCPP_DEBUG_STREAM(this->get_logger(), "Target in camera frame: " << std::endl << target_camera_position);
                                RCLCPP_DEBUG_STREAM(this->get_logger(), "Rotation: " << std::endl << camera_to_kinematic.rotation());
                                RCLCPP_DEBUG_STREAM(this->get_logger(), "Translation: " << std::endl << camera_to_kinematic.translation());    
                                RCLCPP_DEBUG_STREAM(this->get_logger(), "Target in arm base frame: " << std::endl << target_position);                            
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
                if(!sphere_detector->IBTargetPosition(rgb_msg, depth_msg, target_feature)) {
                    RCLCPP_INFO(this->get_logger(), "Target lost!");
                    target_buffer.clear();
                    controller_status = ControllerStatus::NO_TARGET;
                    return;
                }
                else {

                    target_camera_position = sphere_detector->PBTargetPosition(target_feature);
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

    void WeederController::joint_states_callback(
        const sensor_msgs::msg::JointState::SharedPtr msg)
    {

        if (controller_status != ControllerStatus::POSITIONING && controller_status != ControllerStatus::ARM_MOVING) {
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

    
    void WeederController::pose_timer_callback()
    {
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

    void WeederController::pose_callback(geometry_msgs::msg::TransformStamped msg)
    {
        // ------------------- TODO: What do I do of header? ----------------------
        new_pose = tf2::transformToEigen(msg); // convertion out of switch case because i could change message type

        switch (controller_status){

            case ControllerStatus::NO_TARGET:
            {
                // Send initial position
                // TODO: add scanning movement
                send_joint_trajectory(initial_joint_values);
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
                    joints_client->async_cancel_all_goals();
                    // TODO: Altro?
                }
                else {
                    ErrorType error_type;
                    std::vector<double> joint_angles;
                    if (arm_kinematic->computeIK(target_position, joint_angles, error_type)){
                        // RCLCPP_INFO_STREAM(this->get_logger(), "Arm moving to: " << joint_angles[0] << ", " << joint_angles[1]); 
                        send_joint_trajectory(joint_angles);
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
                                RCLCPP_INFO_THROTTLE(this->get_logger(), *(this->get_clock()), message_throttle_ms, "Get closer, target out of reach.");
                                double r = std::sqrt(target_position.x()*target_position.x() + target_position.y()*target_position.y());
                                RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *(this->get_clock()), message_throttle_ms, "r = " << r << " l1 + l2 = " << (l1 + l2));
                                // TODO: print position to check if calculations are correct
                                break;
                            }
                            case ErrorType::EXCLUSION_ZONE:
                            {
                                RCLCPP_INFO_THROTTLE(this->get_logger(), *(this->get_clock()), message_throttle_ms, "Target unreachable, move around obstacles");
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
                    target_buffer.clear();
                    target_feature.setZero();  
                    target_camera_position.setZero();
                    controller_status = ControllerStatus::NO_TARGET;
                    joints_client->async_cancel_all_goals();
                }
                break;
            }

            case ControllerStatus::LASERING:
            {
                if (!new_pose.isApprox(old_pose, pose_threshold)) {
                    controller_status = ControllerStatus::NO_TARGET;
                    RCLCPP_INFO(this->get_logger(), "Mobile robot moved during LASERING");
                    // TODO: cosa si fa qui? è molto sensibile
                }
                break;
            }

        }

        // DO NOT FORGET TO UPDATE POSE
        old_pose = new_pose;
    }

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
        last_joint_angles = joint_angles;
        goal_msg.trajectory.points.clear();
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = joint_angles;
        if (controller_status == ControllerStatus::POSITIONING) {
            point.time_from_start = rclcpp::Duration(std::chrono::milliseconds(trajectory_dt));
        }
        else {
            point.time_from_start = rclcpp::Duration(std::chrono::milliseconds(trajectory_time_ms));
        }
        goal_msg.trajectory.points.push_back(point);

        joints_client->async_send_goal(goal_msg, goal_options);
    }

    void WeederController::result_callback(const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
            {
                if(controller_status == ControllerStatus::ARM_MOVING) {
                    // controller_status = ControllerStatus::POSITIONING;
                    activate_laser(); // skipping control
                }

                break;
            }
                // TODO: handle other cases
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Joint trajectory execution aborted.");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Joint trajectory execution canceled.");
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
        }
        return true;
    }

    void WeederController::control_callback()
    {

        if(controller_status != ControllerStatus::POSITIONING) {
            return;
        }
        else if (target_feature.isZero()) {
            // std::vector<double> joint_goal = WeederController::PBVS_control(target_camera_position);
            // send_joint_trajectory(joint_goal);
            // Pass because PBVS doesn't work, there is an error in measuring Z
            return;
        }
        else {
            double pixel_error = std::sqrt(std::pow(target_feature.x() - desired_feature.x(), 2) + std::pow(target_feature.y() - desired_feature.y(), 2));

            RCLCPP_INFO_STREAM(this->get_logger(), "target feature: " << target_feature.transpose());
            RCLCPP_INFO_STREAM(this->get_logger(), "desired feature: " << desired_feature.transpose());
            RCLCPP_INFO_STREAM(this->get_logger(), "pixel error: " << pixel_error);
            if (pixel_error < control_threshold) {
                activate_laser();
                target_feature.setZero(); 
            }
            else {
                std::vector<double> joint_goal = WeederController::IBVS_control(target_feature);
                send_joint_trajectory(joint_goal);
            }
        }
    }

    std::vector<double> WeederController::PBVS_control(Eigen::Vector3d& target)
    {
        double dt = static_cast<double>(control_dt) / 1000.0;

        // Errore cartesiano nel frame camera
        Eigen::Vector3d e_p;
        if (filtering) {
            e_p = target - desired_position;

            // Smoothing error
            e_p_filtered = beta.cwiseProduct(e_p) + (Eigen::Vector3d(1.0, 1.0, 1.0) - beta).cwiseProduct(e_p_filtered);
            e_p = e_p_filtered;            
        }
        else {
            // Computing error
             e_p = target - desired_position;
        }

        // Velocità camera
        Eigen::Vector3d v_c = - (lambda_PBVS * e_p);

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

        // Jacobiano SCARA (EE wrt base)
        Eigen::Matrix3d J = arm_kinematic->computeScaraJacobian(joint_states);

        // Velocità giunti
        Eigen::Vector3d q_dot = J.inverse() * v_e;

        // Integrazione
        auto q = Eigen::Vector3d(joint_states[0], joint_states[1], 0.0);
        Eigen::Vector3d q_next = q + q_dot * dt;

        RCLCPP_INFO(this->get_logger(), "======================DEBUG PBVS======================");
        RCLCPP_INFO_STREAM(this->get_logger(), "Target position (m): " << target.transpose());
        RCLCPP_INFO_STREAM(this->get_logger(), "Desired position (m): " << desired_position.transpose());
        RCLCPP_INFO_STREAM(this->get_logger(), "Camera velocity (m/s): " << v_c.transpose());
        RCLCPP_INFO_STREAM(this->get_logger(), "EE velocity (m/s): " << v_e.transpose());
        RCLCPP_INFO_STREAM(this->get_logger(), "Joint velocity (rad/s): " << q_dot.transpose());
        RCLCPP_INFO_STREAM(this->get_logger(), "Current joint angles (rad): " << q.transpose());
        RCLCPP_INFO_STREAM(this->get_logger(), "Next joint angles (rad): " << q_next.transpose());
        RCLCPP_INFO(this->get_logger(), "=====================================================");


        return {q_next(0), q_next(1)};
    }

    std::vector<double> WeederController::IBVS_control(Eigen::Vector3d& feature)
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

        Eigen::Vector3d v_c = - (lambda_IBVS * L_pinv * e_s);

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

        // Velocità giunti
        Eigen::Vector3d q_dot = J.inverse() * v_e;

        // Integrazione
        // Potrei aggiungere un terzo joint state per considerare la focale del laser
        auto q = Eigen::Vector3d(joint_states[0], joint_states[1], 0.0);
        Eigen::Vector3d q_next = q + q_dot * dt;

        RCLCPP_INFO(this->get_logger(), "======================DEBUG IBVS======================");
        RCLCPP_INFO_STREAM(this->get_logger(), "Feature error (in pixels): " << e_s.transpose());
        RCLCPP_INFO_STREAM(this->get_logger(), "Camera velocity (m/s): " << v_c.transpose());
        RCLCPP_INFO_STREAM(this->get_logger(), "EE velocity (m/s): " << v_e.transpose());
        RCLCPP_INFO_STREAM(this->get_logger(), "Joint velocity (rad/s): " << q_dot.transpose());
        RCLCPP_INFO_STREAM(this->get_logger(), "Current joint angles (rad): " << q.transpose());
        RCLCPP_INFO_STREAM(this->get_logger(), "Next joint angles (rad): " << q_next.transpose());
        RCLCPP_INFO(this->get_logger(), "=====================================================");

        return {q_next(0), q_next(1)};
    }

    void WeederController::activate_laser()
    {
        controller_status = ControllerStatus::LASERING;
        geometry_msgs::msg::PointStamped lasered_position;
        try {
            // TODO: remove odom and use global fixed frame
            geometry_msgs::msg::TransformStamped arm_pose = tf_buffer->lookupTransform("odom", "kinematic_link", tf2::TimePointZero);
            Eigen::Isometry3d kinematic_to_base = tf2::transformToEigen(arm_pose);
            Eigen::Vector3d absolute_position = kinematic_to_base * target_position;
            lasered_position.point = tf2::toMsg(absolute_position);
            lasered_position.header.stamp = this->now();
            lasered_position.header.frame_id = "odom";
            laser_pub->publish(lasered_position);

        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform odom to kinematic_link: %s", ex.what());
        }                     

        controller_status = ControllerStatus::NO_TARGET;
        return;
    }

} // namespace arm mazzolini

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<arm_mazzolini::WeederController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
