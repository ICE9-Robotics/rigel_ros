cmake_minimum_required(VERSION 2.8.3)
project(rigel_bringup)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch params rviz scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
