#include "arm_mazzolini/sphere_detector.hpp"  

namespace arm_mazzolini
{
    SphereDetector::SphereDetector(int roi_size, int morph_kernel_size, int depth_roi_size)
        : roi_size_(roi_size),
          morph_kernel_size_(morph_kernel_size),
          depth_roi_size_(depth_roi_size)
    {
        // Do something here?
    }

    void SphereDetector::SetCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg)
    {
        fx_ = info_msg->k[0];
        fy_ = info_msg->k[4];
        cx_ = info_msg->k[2];
        cy_ = info_msg->k[5];
    }

    bool SphereDetector::DetectSphere(
        const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
        Eigen::Vector3d &center_position)
    {

        // RGB analysis
        cv::Mat rgb;
        try
        {
            rgb = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::RGB8)->image;
        }
        catch (cv_bridge::Exception &ex)
        {
            RCLCPP_ERROR(rclcpp::get_logger("SphereDetector"), "cv_bridge exception: %s", ex.what());
            return false;
        }
        cv::Mat hsv;
        cv::cvtColor(rgb, hsv, cv::COLOR_RGB2HSV); // RGB to HSV

        cv::Mat mask = createMask(hsv);
        mask = cleanMask(mask);

        // Debug options
        cv::imshow("RGB", rgb);
        cv::imshow("HSV", hsv);
        cv::imshow("mask", mask);
        cv::waitKey(1);

        cv::Point centroid;
        bool found = findLargestBlob(mask, centroid);
        if (!found)
        {
            return false;
        }

        // depth analysis
        cv::Mat depth;
        try
        {
            depth = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding)->image;
        }
        catch (cv_bridge::Exception &ex)
        {
            RCLCPP_ERROR(rclcpp::get_logger("SphereDetector"), "cv_bridge exception: %s", ex.what());
            return false;
        }

        float Z = getDepthMedian(depth, centroid);

        center_position.x() = (centroid.x - cx_) * Z / fx_;
        center_position.y() = (centroid.y - cy_) * Z / fy_;
        center_position.z() = Z;
        // center_poistion is in camera frame, need to convert it
        
        return true;
    }

    cv::Mat SphereDetector::createMask(const cv::Mat &hsv)
    {
        // TODO: change it to green
        cv::Mat mask1, mask2;
        // Hue 0-10
        cv::inRange(hsv, cv::Scalar(0, 100, 50), cv::Scalar(10, 255, 255), mask1);
        // Hue 170-180
        cv::inRange(hsv, cv::Scalar(170, 100, 50), cv::Scalar(180, 255, 255), mask2);
        cv::Mat mask = mask1 | mask2;
        return mask;
    }

    cv::Mat SphereDetector::cleanMask(const cv::Mat &mask)
    {
        // Remove noise and fill holes
        // Maybe it's too slow for this application and second part is unnecessary
        cv::Mat cleaned_mask;
        cv::Mat morph_kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(morph_kernel_size_, morph_kernel_size_));
        cv::morphologyEx(mask, cleaned_mask, cv::MORPH_OPEN, morph_kernel);
        cv::morphologyEx(cleaned_mask, cleaned_mask, cv::MORPH_CLOSE, morph_kernel);
        return cleaned_mask;
    }

    bool SphereDetector::findLargestBlob(const cv::Mat &mask, cv::Point &centroid)
    {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (contours.empty())
        {
            return false;
        }

        // Find the largest contour
        size_t largest_contour_index = 0;
        double largest_area = 0.0;
        for (size_t i = 0; i < contours.size(); ++i)
        {
            double area = cv::contourArea(contours[i]);
            if (area > largest_area)
            {
                largest_area = area;
                largest_contour_index = i;
            }
        }

        // Compute centroid
        cv::Moments mu = cv::moments(contours[largest_contour_index]);
        if (mu.m00 == 0)
        {
            return false;
        }   

        centroid = cv::Point(static_cast<int>(mu.m10 / mu.m00), static_cast<int>(mu.m01 / mu.m00));
        return true;
    }


    float SphereDetector::getDepthMedian(const cv::Mat& depth, const cv::Point& center)
    {
        int half = depth_roi_size_/2;
        std::vector<float> values;
        for (int i = -half; i <= half; ++i)
        {
            for (int j = -half; j <= half; ++j)
            {
                int x = center.x + i;
                int y = center.y + j;
                if (x < 0 || y < 0 || x >= depth.cols || y >= depth.rows)
                    continue;
                float d = depth.at<float>(y,x);
                if (std::isfinite(d) && d > 0)
                    values.push_back(d);
            }
        }
        if (values.empty())
            return 0.0;
        std::nth_element(values.begin(), values.begin()+values.size()/2, values.end());
        return values[values.size()/2];
    }

} // namespace arm_mazzolini