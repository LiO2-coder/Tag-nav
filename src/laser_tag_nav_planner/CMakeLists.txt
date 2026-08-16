cmake_minimum_required(VERSION 3.0.2)
project(laser_tag_nav_planner)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs message_generation nav_msgs roscpp roslib std_msgs tf
)
find_package(jsoncpp REQUIRED)

add_service_files(
  FILES
    PlanPath.srv
)
generate_messages(
  DEPENDENCIES
    nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES laser_tag_nav_planner_core laser_tag_nav_planner_ros
  CATKIN_DEPENDS geometry_msgs message_runtime nav_msgs roscpp roslib std_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

add_library(laser_tag_nav_planner_core
  src/core/json_util.cpp
  src/core/direction.cpp
  src/core/tag_map_loader.cpp
  src/core/connectivity_loader.cpp
  src/core/graph.cpp
  src/core/astar.cpp
)
target_link_libraries(laser_tag_nav_planner_core jsoncpp)

add_library(laser_tag_nav_planner_ros
  src/ros/ros_planner_config.cpp
  src/ros/waypoint_follower.cpp
)
target_include_directories(laser_tag_nav_planner_ros PRIVATE ${catkin_INCLUDE_DIRS})
add_dependencies(laser_tag_nav_planner_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(laser_tag_nav_planner_ros laser_tag_nav_planner_core ${catkin_LIBRARIES})

add_executable(laser_tag_nav_planner_node
  src/planner_node.cpp
)
target_include_directories(laser_tag_nav_planner_node PRIVATE ${catkin_INCLUDE_DIRS})
add_dependencies(laser_tag_nav_planner_node
  ${PROJECT_NAME}_generate_messages_cpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(laser_tag_nav_planner_node
  laser_tag_nav_planner_ros laser_tag_nav_planner_core
  ${catkin_LIBRARIES} jsoncpp
)

install(TARGETS laser_tag_nav_planner_node laser_tag_nav_planner_core laser_tag_nav_planner_ros
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(PROGRAMS
  scripts/tag_connectivity_editor.py
  scripts/tag_nav_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_planner test/test_planner.cpp)
  if(TARGET test_planner)
    target_link_libraries(test_planner laser_tag_nav_planner_core ${catkin_LIBRARIES} jsoncpp)
  endif()
endif()
