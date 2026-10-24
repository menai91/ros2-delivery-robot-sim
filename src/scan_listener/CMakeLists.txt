cmake_minimum_required(VERSION 3.8)
project(scan_listener)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# add_executable: builds a program named scan_listener from our .cpp file.
add_executable(scan_listener src/scan_listener.cpp)

# ament_target_dependencies: links our program against the 2 packages
ament_target_dependencies(scan_listener rclcpp sensor_msgs geometry_msgs)

install(TARGETS scan_listener
  DESTINATION lib/${PROJECT_NAME})


ament_package()
