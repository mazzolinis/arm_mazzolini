#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

class CameraInfoPublisher : public rclcpp::Node
{
public:
  CameraInfoPublisher()
  : Node("camera_info_publisher")
  {
    params_declaration();

    // CameraInfoManager
    cam_info_mgr_ =
      std::make_shared<camera_info_manager::CameraInfoManager>(
        this, camera_name_, camera_config_file_);

    if (!cam_info_mgr_->isCalibrated())
    {
      RCLCPP_ERROR(this->get_logger(), "Camera not calibrated! Current value: %s",camera_config_file_.c_str());
      rclcpp::shutdown();
    }

    // Needed QoS for Gazebo 
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    ci_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, qos);

    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_,
        qos,
        std::bind(&CameraInfoPublisher::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "CameraInfoPublisher running\n"
      "  image_topic:      %s\n"
      "  camera_info_topic:%s\n"
      "  camera_name:      %s",
      image_topic_.c_str(),
      camera_info_topic_.c_str(),
      camera_name_.c_str());
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    sensor_msgs::msg::CameraInfo ci = cam_info_mgr_->getCameraInfo();

    // Synchronize header
    ci.header.stamp = msg->header.stamp;
    ci.header.frame_id = msg->header.frame_id;

    ci_pub_->publish(ci);
  }
  
  void params_declaration(void)
  {
    try{
      this->declare_parameter<std::string>("camera_name", "camera");
      this->declare_parameter<std::string>("camera_config_file", "");
      this->declare_parameter<std::string>("image_topic","/simulated_D435/image");
      this->declare_parameter<std::string>("camera_info_topic","/simulated_D435/camera_info");

      camera_name_ = this->get_parameter("camera_name").as_string();
      camera_config_file_ = this->get_parameter("camera_config_file").as_string();

      // Ensure camera info URL has a valid scheme for camera_info_manager
      if (camera_config_file_.rfind("file://", 0) != 0 && camera_config_file_.rfind("package://", 0) != 0) {
        camera_config_file_ = std::string("file://") + camera_config_file_;
      }
      
      image_topic_ = this->get_parameter("image_topic").as_string();
      camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
    }
    catch (const rclcpp::ParameterTypeException & e)
    {
      RCLCPP_ERROR(this->get_logger(), "Parameter declaration error: %s", e.what());
      RCLCPP_ERROR(this->get_logger(), "Shutting down ...");
      rclcpp::shutdown();
    }
  }

  std::string camera_name_;
  std::string camera_config_file_;
  std::string image_topic_;
  std::string camera_info_topic_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_mgr_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr ci_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraInfoPublisher>());
  rclcpp::shutdown();
  return 0;
}
