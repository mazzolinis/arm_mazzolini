#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include "arm_mazzolini/shared_classes.hpp"

namespace arm_mazzolini
{
class SphereDetector
{
    public:
        SphereDetector(int roi_size = 5, int morph_kernel_size = 5, int depth_roi_size = 5, MaskType mask_type = MaskType::HSV_red);

    // Functions
    void SetCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg);
    std::vector<double> getCameraCenter();
    std::vector<double> getCameraFocals();
    bool PBTargetPosition(
        const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
        Eigen::Vector3d &centroid_position);
    bool IBTargetPosition(
        const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
        Eigen::Vector3d &centroid_feature);
    Eigen::Vector3d PBTargetPosition(const Eigen::Vector3d &centroid_feature);
    bool DetectSphere(
        const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
        cv::Point &centroid,
        double &Z);
    cv::Mat createMask(const cv::Mat &hsv);
    cv::Mat cleanMask(const cv::Mat &mask);
    bool findLargestBlob(const cv::Mat &mask, cv::Point &centroid);
    double getDepthMedian(const cv::Mat &depth, const cv::Point &center);
    void RGBDecomposition(const cv::Mat &rgb, cv::Mat &r, cv::Mat &g, cv::Mat &b);
   
    private:
    // Parameters
    int roi_size_;
    int morph_kernel_size_;
    int depth_roi_size_;
    double fx_, fy_, cx_, cy_;
    MaskType mask_type_;
    double exg_threshold = 0.2; // TODO: tune this parameter
};

} // namespace arm_mazzolini