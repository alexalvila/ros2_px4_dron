cmake_minimum_required(VERSION 3.8)
project(dron_bringup_pkg_mavros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)

include_directories(include)

function(add_dron_node target source)
  add_executable(${target} ${source})
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${target} ${ARGN})
endfunction()

add_dron_node(keyboard_node_mavros src/keyboard_node_mavros.cpp
  rclcpp
  geometry_msgs
  std_msgs
)

add_dron_node(manual_control_node_mavros src/manual_control_node_mavros.cpp
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  mavros_msgs
  tf2
)

add_dron_node(manual_safety_node_mavros src/manual_safety_node_mavros.cpp
  rclcpp
  std_msgs
  sensor_msgs
  mavros_msgs
)

add_dron_node(telemetry_node_mavros src/telemetry_node_mavros.cpp
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  mavros_msgs
)

add_dron_node(tf_odometry_node_mavros src/tf_odometry_node_mavros.cpp
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
)

install(TARGETS
  keyboard_node_mavros
  manual_control_node_mavros
  manual_safety_node_mavros
  telemetry_node_mavros
  tf_odometry_node_mavros
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
