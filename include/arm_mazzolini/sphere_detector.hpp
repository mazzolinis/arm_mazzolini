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
    void SetCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg);
    bool DetectSphere(
        const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
        Eigen::Vector3d &center_position);
    cv::Mat createMask(const cv::Mat &hsv);
    cv::Mat cleanMask(const cv::Mat &mask);
    bool findLargestBlob(const cv::Mat &mask, cv::Point &centroid);
    float getDepthMedian(const cv::Mat &depth, const cv::Point &center);
   
    private:
    // Parameters
    int roi_size_;
    int morph_kernel_size_;
    int depth_roi_size_;
    double fx_, fy_, cx_, cy_;
};

} // namespace arm_mazzolini