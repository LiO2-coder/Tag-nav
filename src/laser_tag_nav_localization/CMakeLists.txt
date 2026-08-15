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
  INCLUDE_DIRS include
  LIBRARIES laser_tag_nav_localization_core laser_tag_nav_localization_ros
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport message_runtime roscpp roslib sensor_msgs std_msgs tf
  DEPENDS OpenCV
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${APRILTAG_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)
link_directories(${APRILTAG_LIBRARY_DIRS})

add_library(laser_tag_nav_localization_core
  src/core/configuration.cpp
  src/core/transform_math.cpp
  src/core/apriltag_recognizer.cpp
  src/core/fusion_engine.cpp
  src/core/pose_temporal_filter.cpp
)
target_link_libraries(laser_tag_nav_localization_core
  ${OpenCV_LIBRARIES} ${APRILTAG_LIBRARIES} jsoncpp
)

add_library(laser_tag_nav_localization_ros
  src/ros/ros_config_loader.cpp
  src/ros/ros_message_adapter.cpp
  src/ros/ros_tf_bridge.cpp
)
target_include_directories(laser_tag_nav_localization_ros PRIVATE ${catkin_INCLUDE_DIRS})
add_dependencies(laser_tag_nav_localization_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(laser_tag_nav_localization_ros
  laser_tag_nav_localization_core ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

add_executable(apriltag_localization_node
  src/apriltag_localization_node.cpp
)
target_include_directories(apriltag_localization_node PRIVATE ${catkin_INCLUDE_DIRS})
add_dependencies(apriltag_localization_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(apriltag_localization_node
  laser_tag_nav_localization_ros laser_tag_nav_localization_core
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${APRILTAG_LIBRARIES} jsoncpp
)

install(TARGETS apriltag_localization_node laser_tag_nav_localization_core laser_tag_nav_localization_ros
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

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_quality test/test_quality.cpp)
  if(TARGET test_quality)
    target_link_libraries(test_quality ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  endif()

  catkin_add_gtest(test_core test/test_core.cpp)
  if(TARGET test_core)
    target_link_libraries(test_core laser_tag_nav_localization_core ${OpenCV_LIBRARIES}
      ${APRILTAG_LIBRARIES} jsoncpp)
  endif()

  catkin_add_gtest(test_temporal_filter test/test_temporal_filter.cpp)
  if(TARGET test_temporal_filter)
    target_link_libraries(test_temporal_filter laser_tag_nav_localization_core ${OpenCV_LIBRARIES}
      ${APRILTAG_LIBRARIES} jsoncpp)
  endif()
endif()
