cmake_minimum_required(VERSION 3.0.2)
project(laser_tag_nav_localization)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge geometry_msgs image_transport message_generation roscpp roslib sensor_msgs std_msgs tf
)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(APRILTAG REQUIRED apriltag)
find_package(jsoncpp REQUIRED)

add_message_files(
  FILES
    CameraBestTag.msg
    CameraBestTagArray.msg
    FusedAprilTagLocalization.msg
)
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport message_runtime roscpp roslib sensor_msgs std_msgs tf
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${APRILTAG_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)
link_directories(${APRILTAG_LIBRARY_DIRS})

add_executable(apriltag_localization_node
  src/apriltag_localization_node.cpp
)
add_dependencies(apriltag_localization_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(apriltag_localization_node
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${APRILTAG_LIBRARIES} jsoncpp
)

install(TARGETS apriltag_localization_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_quality test/test_quality.cpp)
  if(TARGET test_quality)
    target_link_libraries(test_quality ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  endif()
endif()
