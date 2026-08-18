cmake_minimum_required(VERSION 3.8)
project(rm75_fixed_grasp)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rm_ros_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(fixed_grasp_node src/fixed_grasp_node.cpp)
add_executable(turn_knob_node src/turn_knob_node.cpp)
add_executable(qr_grasp_test src/qr_grasp_test.cpp)
ament_target_dependencies(qr_grasp_test rclcpp)

target_compile_features(fixed_grasp_node PUBLIC cxx_std_17)
ament_target_dependencies(fixed_grasp_node
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rm_ros_interfaces
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

target_compile_features(turn_knob_node PUBLIC cxx_std_17)
ament_target_dependencies(turn_knob_node
  control_msgs
  geometry_msgs
  rclcpp
  rclcpp_action
  rm_ros_interfaces
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
)

install(TARGETS fixed_grasp_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS turn_knob_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS qr_grasp_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
