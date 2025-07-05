cmake_minimum_required(VERSION 3.8)
project(autoware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)

include_directories(include)

add_library(
  autoware_interface SHARED
  src/autoware_interface.cpp
)

ament_target_dependencies(
  autoware_interface
  rclcpp
  rclcpp_components
  std_msgs
  can_msgs
  autoware_vehicle_msgs
  autoware_control_msgs
)

rclcpp_components_register_nodes(
  autoware_interface
  "autoware_interface_ns::AutowareInterface"
)

install(TARGETS 
  autoware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
