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

# 1. Attitude Estimator
add_executable(attitude_estimator src/attitude_estimator.cpp)
ament_target_dependencies(attitude_estimator rclcpp sensor_msgs geometry_msgs)

# 2. Stabilator PID Controller
add_executable(stabilator_controller src/stabilator_controller.cpp)
ament_target_dependencies(stabilator_controller rclcpp geometry_msgs std_msgs)

# 3. Landing Gear Controller
add_executable(landing_gear_controller src/landing_gear_controller.cpp)
ament_target_dependencies(landing_gear_controller rclcpp std_msgs)

# 4. Flight Dynamics Node
add_executable(flight_dynamics_node src/flight_dynamics_node.cpp)
ament_target_dependencies(flight_dynamics_node rclcpp geometry_msgs std_msgs)

install(TARGETS
  attitude_estimator
  stabilator_controller
  landing_gear_controller
  flight_dynamics_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()