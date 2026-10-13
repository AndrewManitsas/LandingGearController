cmake_minimum_required(VERSION 3.8)
project(intelligence_domain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Terrain Classifier Node
add_executable(terrain_classifier src/terrain_classifier.cpp)
target_include_directories(terrain_classifier PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(terrain_classifier
  rclcpp
  std_msgs
)

# Deterministic Inference Latency Benchmark Target
add_executable(benchmark_inference src/benchmark_inference.cpp)
target_include_directories(benchmark_inference PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  terrain_classifier
  benchmark_inference
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()