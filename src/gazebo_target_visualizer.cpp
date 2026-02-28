#include "arm_mazzolini/gazebo_target_visualizer.hpp"

GazeboTargetVisualizer::GazeboTargetVisualizer() 
: Node("gazebo_target_visualizer")
{
  get_material_parameter("ambient", ambient_);
  get_material_parameter("diffuse", diffuse_);
  get_material_parameter("specular", specular_);
  get_material_parameter("emissive", emissive_);
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

void GazeboTargetVisualizer::get_material_parameter(const std::string& param_name, std::vector<double>& parameter)
{
  this->declare_parameter("material." + param_name, std::vector<double>());
  parameter = this->get_parameter("material." + param_name).as_double_array();

  if(parameter.size() != 4) {
    RCLCPP_FATAL(this->get_logger(), "Material %s has size %zu, it must be of size 4 (RGBA)", param_name.c_str(), parameter.size());
    rclcpp::shutdown();
  }
  
  for (size_t i = 0; i < parameter.size(); i++) {
    if (parameter[i] < 0.0 || parameter[i] > 1.0) {
      RCLCPP_FATAL(this->get_logger(), "Material %s has value %f at index %zu, it must be in the range [0, 1]", param_name.c_str(), parameter[i], i);
      rclcpp::shutdown();
    }
  } 
}

std::string GazeboTargetVisualizer::color_to_string(const std::vector<double>& c)
{
  std::string oss;
  oss += std::to_string(c[0]) + " " + std::to_string(c[1]) + " " + std::to_string(c[2]) + " " + std::to_string(c[3]);
  return oss;
};


void GazeboTargetVisualizer::target_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{

  RCLCPP_INFO(this->get_logger(), " =========================== TARGET RICEVUTO =========================== ");
  // ensure service is available (not sure if this is correct way to do it)
  while (!spawn_client_->wait_for_service(wait_time)) {
    if (!rclcpp::ok()){
      return;
    }
    RCLCPP_WARN(this->get_logger(), "spawn_entity service not available");
  }

  delete_current_target(); // TODO: add a way to visualize multiple targets, maybe an ID system?
  
  current_target_name_ = std::string("target_sphere_") + std::to_string(this->now().nanoseconds());
  // current_target_name_ = "target_red_sphere";

  auto point = msg->point;
  
  // // Sphere
  // std::string xml;
  // xml += R"(<?xml version="1.0"?>)";
  // xml += R"(<sdf version="1.8"><model name=")";
  // xml += current_target_name_;
  // xml += R"(">)";
  // xml += "<static>true</static>"; 
  // xml += "<pose>";
  // xml += std::to_string(point.x) + " " + std::to_string(point.y) + " " + std::to_string(point.z) + " ";
  // xml += "0 0 0</pose>";
  // xml += R"(
  //     <link name="link">
  //       <!-- Disable gravity for this object -->
  //       <gravity>false</gravity>  

  //       <!-- VISUAL -->
  //       <visual name="visual">
  //         <geometry><sphere><radius>0.02</radius></sphere></geometry>
  //         <material>)";
  // xml += "<ambient>" + color_to_string(ambient_) + "</ambient>";
  // xml += "<diffuse>" + color_to_string(diffuse_) + "</diffuse>";
  // xml += "<specular>" + color_to_string(specular_) + "</specular>";
  // xml += "<emissive>" + color_to_string(emissive_) + "</emissive>";
  // xml += "</material>";
  // xml += "</visual>";

  // xml += R"(
  //       <!-- COLLISION (physics interaction) -->
  //       <collision name="collision">
  //         <geometry><sphere><radius>0.02</radius></sphere></geometry>
  //         <surface>
  //           <contact>
  //             <ode>
  //               <min_depth>0.001</min_depth>
  //               <max_vel>0.0</max_vel>
  //             </ode>
  //           </contact>
  //         </surface>
  //       </collision>
        
  //       <!-- INERTIAL (mass properties) -->
  //       <inertial>
  //         <mass>0.1</mass>  <!-- Small mass -->
  //         <inertia>
  //           <ixx>0.0001</ixx>
  //           <iyy>0.0001</iyy>
  //           <izz>0.0001</izz>
  //         </inertia>
  //       </inertial>
  //     </link>
  //   </model>
  // </sdf>)";

  // Plant (with stl mesh, correct SDF format)
  std::string xml;
  xml += R"(<?xml version="1.0"?>)";
  xml += R"(<sdf version="1.8"><model name=")";
  xml += current_target_name_;
  xml += R"(">)";
  xml += "<static>true</static>";
  xml += "<pose>";
  xml += std::to_string(point.x) + " " + std::to_string(point.y) + " " + std::to_string(point.z) + " ";
  xml += "0 0 1.58</pose>";
  xml += R"(
      <link name="link">
        <!-- Disable gravity for this object -->
        <gravity>false</gravity>

        <!-- VISUAL -->
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>file:///home/simone/Documenti/Uni/arm_mazzolini/install/arm_mazzolini/share/arm_mazzolini/meshes/PlantSproutOriented_millimeter.stl</uri>
              <scale>2.0 2.0 2.0</scale>
            </mesh>
          </geometry>
          <material>)";
  xml += "<ambient>" + color_to_string(ambient_) + "</ambient>";
  xml += "<diffuse>" + color_to_string(diffuse_) + "</diffuse>";
  xml += "<specular>" + color_to_string(specular_) + "</specular>";
  xml += "<emissive>" + color_to_string(emissive_) + "</emissive>";
  xml += "</material>";
  xml += "</visual>";

  xml += R"(
        <!-- COLLISION (physics interaction) -->
        <collision name="collision">
           <geometry>
            <mesh>
              <uri>file:///home/simone/Documenti/Uni/arm_mazzolini/install/arm_mazzolini/share/arm_mazzolini/meshes/PlantSproutOriented_millimeter.stl</uri>
              <scale>2.0 2.0 2.0</scale>
            </mesh>
          </geometry>
          <surface>
            <contact>
              <ode>
                <min_depth>0.001</min_depth>
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
  RCLCPP_INFO_STREAM(this->get_logger(), "Clear target message received: " << std::to_string(msg->data));
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
    RCLCPP_WARN(this->get_logger(), "No current target to delete.");
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
    
    auto future = delete_client_->async_send_request(request, 
      std::bind(&GazeboTargetVisualizer::delete_callback, this, std::placeholders::_1)
      );
    current_target_name_.clear();
  }
}

void GazeboTargetVisualizer::spawn_callback(rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedFuture future)
{
    auto response = future.get();
    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Target spawned successfully!");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to spawn target");
    }
}

void GazeboTargetVisualizer::delete_callback(rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedFuture future)
{
    auto response = future.get();
    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Target deleted successfully!");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to delete target");
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