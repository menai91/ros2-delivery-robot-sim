cmake_minimum_required(VERSION 3.5)
project(ros2_project)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch meshes rviz worlds
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
