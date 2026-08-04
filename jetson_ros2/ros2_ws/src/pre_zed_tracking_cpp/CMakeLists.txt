cmake_minimum_required(VERSION 3.8)
project(pre_zed_tracking_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(simple_tracker src/tracker.cpp)
target_include_directories(simple_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(simple_tracker_node src/simple_tracker_node.cpp)
target_link_libraries(simple_tracker_node simple_tracker)
target_compile_definitions(simple_tracker_node PRIVATE BOOST_BIND_GLOBAL_PLACEHOLDERS)
ament_target_dependencies(simple_tracker_node rclcpp std_msgs)

add_executable(track_depth_fusion_node src/track_depth_fusion_node.cpp)
target_compile_definitions(
  track_depth_fusion_node
  PRIVATE BOOST_BIND_GLOBAL_PLACEHOLDERS
)
ament_target_dependencies(
  track_depth_fusion_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS simple_tracker
  EXPORT export_simple_tracker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(
  TARGETS simple_tracker_node track_depth_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tracker test/test_tracker.cpp)
  target_link_libraries(test_tracker simple_tracker)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_targets(export_simple_tracker)
ament_package()
