#pragma once

#include "arm_mazzolini/standard_detector.hpp"

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
    class SphereDetector : public StandardDetector
    {
        public:
            SphereDetector(int roi_size = 10, int morph_kernel_size = 10, int depth_roi_size = 20, DetectorType mask_type = DetectorType::HSV_red);

        // Functions
        void SetCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg) override;
        std::vector<double> getCameraCenter() override;
        std::vector<double> getCameraFocals() override;
        bool PBTargetPosition(
            const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
            const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
            Eigen::Vector3d &centroid_position) override;
        bool IBTargetPosition(
            const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
            const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
            Eigen::Vector3d &centroid_feature) override;
        Eigen::Vector3d PBTargetPosition(const Eigen::Vector3d &centroid_feature) override;

        private:
        // Parameters
        int roi_size_;
        int morph_kernel_size_;
        int depth_roi_size_;
        double fx_, fy_, cx_, cy_;
        DetectorType mask_type_;
        double exg_threshold = 0.2; // TODO: tune this parameter
        double area_threshold = 100.0; // TODO: tune this parameter
        bool DetectSphere(
            const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
            const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
            cv::Point &centroid,
            double &Z);
        cv::Mat createMask(const cv::Mat &hsv);
        cv::Mat cleanMask(const cv::Mat &mask);
        bool findLargestBlob(const cv::Mat &mask, cv::Point &centroid);
        bool findPlantCentroid(const cv::Mat &mask, cv::Point &centroid);
        double getDepthMedian(const cv::Mat &depth, const cv::Point &center);
        void RGBDecomposition(const cv::Mat &rgb, cv::Mat &r, cv::Mat &g, cv::Mat &b);
    };

} // namespace arm_mazzolini