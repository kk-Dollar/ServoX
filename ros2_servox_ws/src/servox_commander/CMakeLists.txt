cmake_minimum_required(VERSION 3.8)
project(servox_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(test_moveit src/test_moveit.cpp)
ament_target_dependencies(test_moveit
  rclcpp
  moveit_ros_planning_interface
)

add_executable(commander src/commander.cpp)
ament_target_dependencies(commander
  rclcpp
  moveit_ros_planning_interface
  example_interfaces
  std_msgs
  sensor_msgs
  geometry_msgs
)

install(TARGETS 
  test_moveit
  commander
  DESTINATION lib/${PROJECT_NAME})


ament_package()
