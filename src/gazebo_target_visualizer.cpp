#include "arm_mazzolini/gazebo_target_visualizer.hpp"

GazeboTargetVisualizer::GazeboTargetVisualizer() 
: Node("gazebo_target_visualizer")
{
  spawn_client_ = this->create_client<ros_gz_interfaces::srv::SpawnEntity>("/world/default/create");
  delete_client_ = this->create_client<ros_gz_interfaces::srv::DeleteEntity>("/world/default/remove");
  
  target_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/target_position", 10,
    std::bind(&GazeboTargetVisualizer::target_callback, this, std::placeholders::_1)
    );
  
  clear_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/clear_target", 10,
    std::bind(&GazeboTargetVisualizer::clear_callback, this, std::placeholders::_1)
    );
}


void GazeboTargetVisualizer::target_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  // ensure service is available (not sure if this is correct way to do it)
  while (!spawn_client_->wait_for_service(wait_time)) {
    if (!rclcpp::ok()){
      return;
    }
    RCLCPP_WARN(this->get_logger(), "spawn_entity service not available");
  }

  delete_current_target(); // TODO: add a way to visualize multiple targets, maybe an ID system?
  
  current_target_name_ = std::string("target_sphere_") + std::to_string(this->now().nanoseconds());
  
  auto point = msg->point;

  // RCLCPP_INFO(this->get_logger(), "Spawning new target at [%.2f, %.2f, %.2f]", point.x, point.y, point.z);

  std::string xml;
  xml += R"(<?xml version="1.0"?>)";
  xml += R"(<sdf version="1.8"><model name=")";
  xml += current_target_name_;
  xml += R"(">)";
  xml += "<static>true</static>"; 
  xml += "<pose>";
  xml += std::to_string(point.x) + " " + std::to_string(point.y) + " " + std::to_string(point.z);
  xml += R"( 0 0 0</pose>
      <link name="link">
        <gravity>false</gravity>  <!-- Disable gravity for this object -->
        
        <!-- VISUAL (what you see) -->
        <visual name="visual">
          <geometry><sphere><radius>0.03</radius></sphere></geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
            <specular>1 0.5 0.5 1</specular>
            <emissive>0.5 0 0 1</emissive>
          </material>
        </visual>
        
        <!-- COLLISION (physics interaction) -->
        <collision name="collision">
          <geometry><sphere><radius>0.01</radius></sphere></geometry>
          <surface>
            <contact>
              <ode>
                <min_depth>0.01</min_depth>
                <max_vel>0.0</max_vel>
              </ode>
            </contact>
          </surface>
        </collision>
        
        <!-- INERTIAL (mass properties) -->
        <inertial>
          <mass>0.1</mass>  <!-- Small mass -->
          <inertia>
            <ixx>0.0001</ixx>
            <iyy>0.0001</iyy>
            <izz>0.0001</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </sdf>)";

  auto request = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
  request->entity_factory.name = current_target_name_;
  request->entity_factory.sdf = xml;

  request->entity_factory.pose.position.x = point.x;
  request->entity_factory.pose.position.y = point.y;
  request->entity_factory.pose.position.z = point.z;
  request->entity_factory.pose.orientation.x = 0.0;
  request->entity_factory.pose.orientation.y = 0.0;
  request->entity_factory.pose.orientation.z = 0.0;
  request->entity_factory.pose.orientation.w = 1.0;
  request->entity_factory.relative_to = "odom";

  // spawn_client_->async_send_request(request);
  auto future = spawn_client_->async_send_request(request,
    std::bind(&GazeboTargetVisualizer::spawn_callback, this, std::placeholders::_1)
    );
}

void GazeboTargetVisualizer::clear_callback(const std_msgs::msg::Bool::SharedPtr msg)
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
  if (current_target_name_.empty()) {
    // RCLCPP_WARN(this->get_logger(), "No current target to delete.");
    return;
  }
  else if (!delete_client_->wait_for_service(wait_time)) {
    RCLCPP_WARN(this->get_logger(), "Delete_entity service not available.");
    return;
  }
  else {
    auto request = std::make_shared<ros_gz_interfaces::srv::DeleteEntity::Request>();
    request->entity.name = current_target_name_;
    request->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
    
    delete_client_->async_send_request(request);
    current_target_name_.clear();
  }
}

void GazeboTargetVisualizer::spawn_callback(
    rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedFuture future)
{
    auto response = future.get();
    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Target spawned successfully!");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to spawn target");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GazeboTargetVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}