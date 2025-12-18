// TOD0: check this code, node automatically generated

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

using namespace std::chrono_literals;

class CameraInfoPublisher : public rclcpp::Node
{
public:
  CameraInfoPublisher()
  : Node("camera_info_publisher")
  {
    // param defaults
    this->declare_parameter<std::string>("camera_name", "camera");
    this->declare_parameter<std::string>("camera_info_url", ""); // e.g. file:///path/to/camera_info.yaml
    this->declare_parameter<std::string>("image_topic", "/camera/left/image_raw");
    this->declare_parameter<std::string>("camera_info_topic", "/camera/left/camera_info");
    this->declare_parameter<int>("queue_size", 10);

    camera_name_ = this->get_parameter("camera_name").as_string();
    camera_info_url_ = this->get_parameter("camera_info_url").as_string();
    image_topic_ = this->get_parameter("image_topic").as_string();
    camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
    queue_size_ = this->get_parameter("queue_size").as_int();

    // Create CameraInfoManager
    // Note: constructor used by many ROS2 drivers: CameraInfoManager(node, camera_name, camera_info_url)
    cam_info_mgr_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_, camera_info_url_);

    // If manager has a valid calibration loaded, ok; if not, it still works (publishes default CameraInfo)
    if (cam_info_mgr_->isCalibrated()) {
      RCLCPP_INFO(this->get_logger(), "CameraInfoManager: calibration loaded for '%s'", camera_name_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "CameraInfoManager: no calibration loaded for '%s' (publishing default CameraInfo), url='%s'",
                  camera_name_.c_str(), camera_info_url_.c_str());
    }

    // create publisher for CameraInfo
    ci_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, rclcpp::QoS(rclcpp::KeepLast(queue_size_)));

    // subscribe to image topic
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::QoS(rclcpp::KeepLast(queue_size_)).best_effort(),
      std::bind(&CameraInfoPublisher::image_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "CameraInfoPublisher ready: image_topic='%s' -> camera_info_topic='%s' (camera_name='%s')",
                image_topic_.c_str(), camera_info_topic_.c_str(), camera_name_.c_str());
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // obtain camera info from manager
    sensor_msgs::msg::CameraInfo ci = cam_info_mgr_->getCameraInfo();

    // stamp & frame must match incoming image
    ci.header.stamp = msg->header.stamp;
    ci.header.frame_id = msg->header.frame_id;

    ci_pub_->publish(ci);
  }

  // params
  std::string camera_name_;
  std::string camera_info_url_;
  std::string image_topic_;
  std::string camera_info_topic_;
  int queue_size_;

  // camera info manager
  std::unique_ptr<camera_info_manager::CameraInfoManager> cam_info_mgr_;

  // ros entities
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr ci_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraInfoPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
