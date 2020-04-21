cmake_minimum_required(VERSION 2.8.3)
project(laser_scan_processor)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  LineSegment.msg
  LineSegmentList.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs message_runtime roscpp sensor_msgs visualization_msgs
)

## Declare a C++ library
add_library(
  ${PROJECT_NAME}
   src/status.cpp
   src/laser_line_extraction/line.cpp
   src/laser_line_extraction/line_extraction.cpp
   src/laser_line_extraction/line_extraction_ros.cpp
   src/laser_door_detection/door.cpp
   src/laser_door_detection/door_detection.cpp
   src/laser_door_detection/door_detection_ros.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

add_dependencies(
  ${PROJECT_NAME}
   ${catkin_EXPORTED_TARGETS}
)

add_executable(processor_node src/processor_node.cpp)
target_link_libraries(processor_node ${PROJECT_NAME})

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${EIGEN3_INCLUDE_DIRS})

# catkin_add_gtest(${PROJECT_NAME}-test test/test_laser_scan_processor.cpp)
