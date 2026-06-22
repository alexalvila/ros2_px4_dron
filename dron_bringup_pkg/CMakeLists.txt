cmake_minimum_required(VERSION 3.8)
project(dron_bringup_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)

# Incluir headers
include_directories(include)

# Lista de dependencias comunes
set(COMMON_DEPENDENCIES
  rclcpp
  px4_msgs
  geometry_msgs
  std_msgs
)

# Función para añadir nodos fácilmente
function(add_dron_node target source)
  add_executable(${target} ${source})
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${target} ${ARGN})
endfunction()

# Nodos
add_dron_node(keyboard_node src/keyboard_node.cpp ${COMMON_DEPENDENCIES})
add_dron_node(manual_control_node src/manual_control_node.cpp ${COMMON_DEPENDENCIES})
add_dron_node(manual_safety_node src/manual_safety_node.cpp ${COMMON_DEPENDENCIES})
add_dron_node(telemetry_node src/telemetry_node.cpp ${COMMON_DEPENDENCIES})

# Nodo con dependencias extra
add_dron_node(tf_odometry_node src/tf_odometry_node.cpp
  rclcpp
  geometry_msgs
  nav_msgs
  px4_msgs
  tf2
  tf2_ros
)

# Instalación de ejecutables
install(TARGETS
  keyboard_node
  manual_control_node
  manual_safety_node
  telemetry_node
  tf_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

# Instalación de headers y carpetas
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