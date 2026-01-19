cmake_minimum_required(VERSION 3.8)
project(uirover_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Build MT Velocity Controller plugin
add_library(mt_velocity_controller SHARED
  plugins/mt_velocity_controller.cpp
)

target_include_directories(mt_velocity_controller PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/plugins>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mt_velocity_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  geometry_msgs
  std_msgs
)

# Build MT Gazebo Odometry plugin
add_library(mt_gazebo_odometry SHARED
  plugins/mt_gazebo_odometry.cpp
)

target_include_directories(mt_gazebo_odometry PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/plugins>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mt_gazebo_odometry
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  geometry_msgs
  nav_msgs
  sensor_msgs
  gazebo_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Export plugins
pluginlib_export_plugin_description_file(controller_interface mt_velocity_controller_plugin.xml)
pluginlib_export_plugin_description_file(controller_interface mt_gazebo_odometry_plugin.xml)

# Install plugin libraries
install(TARGETS mt_velocity_controller mt_gazebo_odometry
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install directories
install(
  DIRECTORY urdf xacro launch rviz config dae meshes worlds models maps plugins
  DESTINATION share/${PROJECT_NAME}
)

# Install plugin descriptions
install(FILES 
  mt_velocity_controller_plugin.xml
  mt_gazebo_odometry_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()