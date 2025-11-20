#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class TargetSpawner : public rclcpp::Node
{
    public:
        TargetSpawner();
}