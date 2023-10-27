cmake_minimum_required(VERSION 3.5)
project(rigel_ros)
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch params scripts rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
