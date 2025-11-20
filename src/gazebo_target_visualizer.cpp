#include "arm_mazzolini/gazebo_target_visualizer.hpp"

GazeboTargetVisualizer::GazeboTargetVisualizer() 
: Node("gazebo_target_visualizer")
{
  spawn_client_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
  delete_client_ = this->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");
  
  target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
    "/target_position", 10,
    std::bind(&GazeboTargetVisualizer::target_callback, this, std::placeholders::_1)
    );
  
  clear_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/clear_target", 10,
    std::bind(&GazeboTargetVisualizer::clear_callback, this, std::placeholders::_1)
    );
}


void GazeboTargetVisualizer::target_callback(const geometry_msgs::msg::Point::SharedPtr msg)
{
  delete_current_target(); // TODO: add a way to visualize multiple targets, maybe an ID system?
  
  current_target_name_ = "target_sphere_" + std::to_string(this->now().nanoseconds());
  
  auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
  request->name = current_target_name_;
  request->xml = R"(
    <?xml version="1.0"?>
    <sdf version="1.7">
      <model name=")" + current_target_name_ + R"(">
        <pose>)" + std::to_string(msg->x) + " " + std::to_string(msg->y) + " " + std::to_string(msg->z) + R"( 0 0 0</pose>
        <link name="link">
          <visual name="visual">
            <geometry>
              <sphere><radius>0.05</radius></sphere>
            </geometry>
            <material>
              <ambient>1 0 0 0.8</ambient>
              <diffuse>1 0 0 0.8</diffuse>
            </material>
          </visual>
        </link>
      </model>
    </sdf>
  )";
  
  spawn_client_->async_send_request(request);
}

void GazeboTargetVisualizer::clear_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
  if (msg->data == true)
    {
      delete_current_target();
    }
  else
    {
      RCLCPP_WARN(this->get_logger(), "RICEVUTO FALSE: Cosa si fa???");
      // TODO: Change this part
    }
}

void GazeboTargetVisualizer::delete_current_target()
{
    if (!current_target_name_.empty()) {
        auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
        request->name = current_target_name_;
        delete_client_->async_send_request(request);
        current_target_name_.clear();
    }
}