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

add_executable(attitude_estimator src/attitude_estimator.cpp)
ament_target_dependencies(attitude_estimator
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

install(TARGETS
  attitude_estimator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()