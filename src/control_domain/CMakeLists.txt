cmake_minimum_required(VERSION 3.8)
project(control_domain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Task 2.1: Attitude Estimator Node
add_executable(attitude_estimator src/attitude_estimator.cpp)
ament_target_dependencies(attitude_estimator
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

# Task 2.2: Stabilator PID Controller Node
add_executable(stabilator_controller src/stabilator_controller.cpp)
ament_target_dependencies(stabilator_controller
  rclcpp
  geometry_msgs
  std_msgs
)

# Task 2.3: Landing Gear Actuator Controller Node
add_executable(landing_gear_controller src/landing_gear_controller.cpp)
ament_target_dependencies(landing_gear_controller
  rclcpp
  std_msgs
)

install(TARGETS
  attitude_estimator
  stabilator_controller
  landing_gear_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()