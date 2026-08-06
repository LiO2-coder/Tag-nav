cmake_minimum_required(VERSION 3.0.2)
project(laser-tag_nav_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".factory.world.swp" EXCLUDE
)

install(
  PROGRAMS scripts/generate_apriltag_floor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
