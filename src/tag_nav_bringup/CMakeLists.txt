cmake_minimum_required(VERSION 3.0.2)
project(tag_nav_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY config launch maps rviz worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  PROGRAMS
    scripts/generate_apriltag_floor.py
    scripts/generate_factory_map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
