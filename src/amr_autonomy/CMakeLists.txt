cmake_minimum_required(VERSION 3.8)
project(amr_autonomy)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(amr_navigation REQUIRED)

add_executable(bt_runner src/bt_runner.cpp)

add_library(amr_autonomy_nodes
    src/bt_move_to_position.cpp
    src/bt_pick_up_item.cpp
    src/bt_drop_off_item.cpp)

target_include_directories(amr_autonomy_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(bt_runner rclcpp behaviortree_cpp)
ament_target_dependencies(amr_autonomy_nodes rclcpp behaviortree_cpp)

target_link_libraries(amr_autonomy_nodes amr_navigation::amr_navigation_lib)
target_link_libraries(bt_runner amr_autonomy_nodes)

install(TARGETS bt_runner DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY trees config DESTINATION share/${PROJECT_NAME})

ament_package()
