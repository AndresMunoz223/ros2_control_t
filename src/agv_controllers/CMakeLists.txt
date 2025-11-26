cmake_minimum_required(VERSION 3.8)
project(agv_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_interface REQUIRED) 
find_package(tf2_ros REQUIRED)

include_directories(include)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/agv_forward_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_lifecycle pluginlib std_msgs geometry_msgs controller_interface tf2_ros)

pluginlib_export_plugin_description_file(controller_interface rov_controller.xml)


install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)


install(DIRECTORY include
  DESTINATION include
)


ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  pluginlib
  std_msgs
  geometry_msgs
  controller_interface
)

ament_package()
