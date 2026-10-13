cmake_minimum_required(VERSION 3.8)
project(sensing_domain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(sensor_processor src/sensor_processor.cpp)
ament_target_dependencies(sensor_processor
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

install(TARGETS
  sensor_processor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()