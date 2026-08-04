cmake_minimum_required(VERSION 3.0.2)
project(laser-tag_nav_description)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY config launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
