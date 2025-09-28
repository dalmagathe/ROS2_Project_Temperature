cmake_minimum_required(VERSION 3.8)
project(tpr_ctrl)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(get_temp src/node_get_temp.cpp)
ament_target_dependencies(get_temp rclcpp std_msgs)

add_executable(controller src/node_controller.cpp)
ament_target_dependencies(controller rclcpp std_msgs)

install(TARGETS
  get_temp
  controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
