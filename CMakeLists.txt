# ==============================================================================
#                             PROJECT CONFIGURATION                             
# ==============================================================================

cmake_minimum_required(VERSION 3.12)
project(arm_mazzolini)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
  # add_compile_options(-Wall)
endif()


# ==============================================================================
#                               FIND DEPENDENCIES                               
# ==============================================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ros_gz REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2)
find_package(tf2_ros)
find_package(tf2_eigen)

# ==============================================================================
#                                   ADD NODE                                   
# ==============================================================================

add_executable(kinematic_node src/kinematic_node.cpp)
add_executable(gazebo_target_visualizer src/gazebo_target_visualizer.cpp)
add_executable(rviz_target_visualizer src/rviz_target_visualizer.cpp)
add_executable(target_spawner src/target_spawner.cpp)

set(PROJECT_DEPENDENCIES
  rclcpp
  rclcpp_action
  control_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  ros_gz
  ros_gz_interfaces
  gazebo_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
)

ament_target_dependencies(
  kinematic_node PUBLIC
  ${PROJECT_DEPENDENCIES}
)
ament_target_dependencies(
  gazebo_target_visualizer PUBLIC
  ${PROJECT_DEPENDENCIES}
)
ament_target_dependencies(
  rviz_target_visualizer PUBLIC
  ${PROJECT_DEPENDENCIES}
)
ament_target_dependencies(
  target_spawner PUBLIC
  ${PROJECT_DEPENDENCIES}
)

target_include_directories(kinematic_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(gazebo_target_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(rviz_target_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(target_spawner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  DIRECTORY launch urdf config rviz meshes worlds
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS kinematic_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS gazebo_target_visualizer
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS rviz_target_visualizer
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS target_spawner
  DESTINATION lib/${PROJECT_NAME}
)
# ============================================================================== #

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
