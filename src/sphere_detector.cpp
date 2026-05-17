#include "arm_mazzolini/sphere_detector.hpp"  

namespace arm_mazzolini
{
    SphereDetector::SphereDetector(int roi_size, int morph_kernel_size, int depth_roi_size, MaskType mask_type)
        : roi_size_(roi_size),
          morph_kernel_size_(morph_kernel_size),
          depth_roi_size_(depth_roi_size),
          mask_type_(mask_type)
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

    std::vector<double> SphereDetector::getCameraCenter()
    {
        return {cx_, cy_};
    }

    std::vector<double> SphereDetector::getCameraFocals()
    {
        return {fx_, fy_};
    }

    bool SphereDetector::IBTargetPosition(
        const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
        Eigen::Vector3d &centroid_feature)
    {
        cv::Point centroid;
        double Z;
        if (!DetectSphere(rgb_msg, depth_msg, centroid, Z))
        {
            return false;
        }
        else
        {
            // This is a feature, not a position, coordinates are percentages of image size with [0,0] at the center
            centroid_feature.x() = (centroid.x - cx_) / fx_;
            centroid_feature.y() = (centroid.y - cy_) / fy_;
            centroid_feature.z() = Z;
            return true;
        }
    }

    bool SphereDetector::PBTargetPosition(
        const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
        Eigen::Vector3d &centroid_position)
    {
        cv::Point centroid;
        double Z;
        if (!DetectSphere(rgb_msg, depth_msg, centroid, Z))
        {
            return false;
        }
        else
        {
            // Coordinates are in meters
            centroid_position.x() = (centroid.x - cx_) * Z / fx_;
            centroid_position.y() = (centroid.y - cy_) * Z / fy_;
            centroid_position.z() = Z;
            return true;
        }
    }

    Eigen::Vector3d SphereDetector::PBTargetPosition(const Eigen::Vector3d &centroid_feature)
    {
        // This function is used not to evaluate again the images
        Eigen::Vector3d centroid_position;
        centroid_position.x() = centroid_feature.x() * centroid_feature.z();
        centroid_position.y() = centroid_feature.y() * centroid_feature.z();
        centroid_position.z() = centroid_feature.z();
        return centroid_position;
    }
    

    bool SphereDetector::DetectSphere(
        const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
        cv::Point &centroid,
        double &Z)
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

        cv::Mat mask = createMask(rgb);
        mask = cleanMask(mask);

        // bool found = findLargestBlob(mask, centroid);
        bool found = findPlantCentroid(mask, centroid);
        if (!found)
        {
            return false;
        }

        // Depth analysis
        cv::Mat depth;
        try
        {
            cv::Mat depth_raw = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding)->image;

            if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
            {
                // Gazebo simulated camera
                depth = depth_raw;
            }
            else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
            {
                // Real camera
                depth_raw.convertTo(depth, CV_32F, 0.001);
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("SphereDetector"),
                            "Unsupported depth encoding: %s", depth_msg->encoding.c_str());
                return false;
            }
        }
        catch (cv_bridge::Exception &ex)
        {
            RCLCPP_ERROR(rclcpp::get_logger("SphereDetector"), "cv_bridge exception: %s", ex.what());
            return false;
        }

        Z = getDepthMedian(depth, centroid);

        return true;
    }

    void SphereDetector::RGBDecomposition(const cv::Mat &rgb, cv::Mat &r, cv::Mat &g, cv::Mat &b)
    {
        cv::Mat img_float;
        rgb.convertTo(img_float, CV_32FC3);

        std::vector<cv::Mat> channels(3);
        cv::split(img_float, channels);

        cv::Mat sum = channels[0] + channels[1] + channels[2] + 1e-6; // Avoid division by zero 
        r = channels[0] / sum;
        g = channels[1] / sum;
        b = channels[2] / sum;
    }

    cv::Mat SphereDetector::createMask(const cv::Mat &rgb)
    {
        // Core part of the library
        cv::Mat mask;
        
        switch (mask_type_)
        {
            case MaskType::HSV_red:
            {
                cv::Mat hsv;
                cv::cvtColor(rgb, hsv, cv::COLOR_RGB2HSV); // RGB to HSV
                
                cv::Mat mask1, mask2;
                // Hue 0-10
                cv::inRange(hsv, cv::Scalar(0, 100, 50), cv::Scalar(10, 255, 255), mask1);
                // Hue 170-180
                cv::inRange(hsv, cv::Scalar(170, 100, 50), cv::Scalar(180, 255, 255), mask2);
                mask = mask1 | mask2;
                break;
            }

            case MaskType::ExG_threshold:
            {
                cv::Mat r, g, b;
                RGBDecomposition(rgb, r, g, b);
                cv::Mat exg = 2 * g - r - b;
                cv::threshold(exg, mask, exg_threshold, 255, cv::THRESH_BINARY);
                mask.convertTo(mask, CV_8U);
               
                break;
            }

            case MaskType::ExG_Otsu:
            {
                cv::Mat r, g, b;
                RGBDecomposition(rgb, r, g, b);
                cv::Mat exg = 2 * g - r - b;

                // This method needs normalized matrixes and conversion to 8U
                cv::Mat exg_norm;
                cv::normalize(exg, exg_norm, 0, 255, cv::NORM_MINMAX); 
                    exg_norm.convertTo(exg_norm, CV_8U);
                    cv::threshold(exg_norm, mask, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
                break;
            }

            case MaskType::ExGR:
            {
                cv::Mat r, g, b;
                RGBDecomposition(rgb, r, g, b);
                cv::Mat exg = 2 * g - r - b;
                cv::Mat exr = 1.4 * r - g;
                cv::Mat exgr = exg - exr; // Could be done in a single step but it's clearer this way

                cv::threshold(exgr, mask, 0.0, 255, cv::THRESH_BINARY);
                mask.convertTo(mask, CV_8U);
                break;
            }
        }
        
        // Do I need to add something here?
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

    bool SphereDetector::findPlantCentroid(const cv::Mat &mask, cv::Point &centroid)
    {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (contours.empty())
        {
            return false;
        }

        double sum_x = 0.0;
        double sum_y = 0.0;
        double total_area = 0.0;

        for (const auto& contour : contours)
        {
            double area = cv::contourArea(contour);
            if (area < area_threshold)
                continue;

            cv::Moments mu = cv::moments(contour);
            if (mu.m00 == 0)
                continue;

            double cx = mu.m10 / mu.m00;
            double cy = mu.m01 / mu.m00;

            sum_x += cx * area;
            sum_y += cy * area;
            total_area += area;
        }

        if (total_area == 0.0)
            return false;

        centroid.x = static_cast<int>(sum_x / total_area);
        centroid.y = static_cast<int>(sum_y / total_area);

        return true;
    }

    double SphereDetector::getDepthMedian(const cv::Mat& depth, const cv::Point& center)
    {
        int half = depth_roi_size_/2;
        std::vector<double> values;
        for (int i = -half; i <= half; i++)
        {
            for (int j = -half; j <= half; j++)
            {
                int x = center.x + i;
                int y = center.y + j;
                if (x < 0 || y < 0 || x >= depth.cols || y >= depth.rows)
                    continue;
                double d = static_cast<double>(depth.at<float>(y,x));  // Note: depth.at<float>(y,x) because depth is CV_32F
                if (std::isfinite(d) && d > 0.02)
                    values.push_back(d);
            }
        }
        if (values.empty())
            return 0.0;
        std::nth_element(values.begin(), values.begin()+values.size()/2, values.end());
        return values[values.size()/2];
    }

} // namespace arm_mazzolini