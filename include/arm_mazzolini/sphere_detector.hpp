#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>

namespace arm_mazzolini
{
class SphereDetector
{
    public:
        SphereDetector(int roi_size = 5, int morph_kernel_size = 5, int depth_roi_size = 5);

    // Functions
    bool DetectSphere(
        const sensor_msgs::msg::Image::SharedPtr &rgb_msg,
        const sensor_msgs::msg::Image::SharedPtr &depth_msg,
        Eigen::Vector3d &center_position);
    cv::Mat createMask(const cv::Mat &hsv);
    cv::Mat cleanMask(const cv::Mat &mask);
    bool findLargestBlob(const cv::Mat &mask, cv::Point &centroid);
    float getDepthMedian(const cv::Mat &depth, const cv::Point &center, int roi_size);
   
    private:
    // Parameters
    int roi_size_;
    int morph_kernel_size_;
    int depth_roi_size_;
};

} // namespace arm_mazzolini