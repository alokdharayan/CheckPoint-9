cmake_minimum_required(VERSION 3.8)
project(attach_shelf)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================
# Find dependencies
# =====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# =====================
# Service generation
# =====================
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToLoading.srv"
)

# =====================
# Task 1 executable
# =====================
add_executable(pre_approach src/pre_approach.cpp)
ament_target_dependencies(pre_approach
  rclcpp
  sensor_msgs
  geometry_msgs
)

# =====================
# Task 2 executables
# =====================
add_executable(approach_service_server src/approach_service_server.cpp)
ament_target_dependencies(approach_service_server
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
)
rosidl_target_interfaces(approach_service_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

add_executable(pre_approach_v2 src/pre_approach_v2.cpp)
ament_target_dependencies(pre_approach_v2
  rclcpp
)
rosidl_target_interfaces(pre_approach_v2
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# =====================
# Install targets
# =====================
install(TARGETS
  pre_approach
  approach_service_server
  pre_approach_v2
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
