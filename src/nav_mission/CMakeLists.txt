cmake_minimum_required(VERSION 3.8)
project(nav_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(go_to_pose src/go_to_pose.cpp)
add_executable(patrol src/patrol.cpp src/waypoint_loader.cpp)

target_include_directories(go_to_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(patrol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(go_to_pose rclcpp rclcpp_action nav2_msgs geometry_msgs)

ament_target_dependencies(patrol rclcpp rclcpp_action nav2_msgs geometry_msgs)

install(TARGETS go_to_pose
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS patrol
  DESTINATION lib/${PROJECT_NAME})

ament_package()
