cmake_minimum_required(VERSION 3.8)
project(adaptive_pure_pursuit)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(include ${EIGEN3_INCLUDE_DIRS})

# Add executables
add_executable(adaptive_pure_pursuit_node src/adaptive_pure_pursuit_node.cpp)
add_executable(ftg_node src/ftg.cpp)
add_executable(obs_node src/obs_det.cpp)
add_executable(obstacle_avoidance_node src/mux_control_head.cpp)


# Link dependencies
ament_target_dependencies(adaptive_pure_pursuit_node
  rclcpp
  visualization_msgs
  nav_msgs
  std_msgs
  ackermann_msgs
  tf2_ros
  geometry_msgs
  tf2
  Eigen3
)
ament_target_dependencies(obs_node
  rclcpp
  visualization_msgs
  nav_msgs
  std_msgs
  ackermann_msgs
  tf2_ros
  geometry_msgs
  tf2
  Eigen3
  sensor_msgs
)
ament_target_dependencies(ftg_node
  rclcpp
  visualization_msgs
  nav_msgs
  std_msgs
  ackermann_msgs
  geometry_msgs
  sensor_msgs
)
ament_target_dependencies(obstacle_avoidance_node
  rclcpp
  visualization_msgs
  nav_msgs
  std_msgs
  ackermann_msgs
  tf2_ros
  geometry_msgs
  tf2
  Eigen3
  sensor_msgs
)


# Install targets
install(TARGETS adaptive_pure_pursuit_node ftg_node obs_node obstacle_avoidance_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch  
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
