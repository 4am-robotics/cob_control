cmake_minimum_required(VERSION 2.8.3)
project(cob_frame_tracker)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs cob_srvs control_toolbox dynamic_reconfigure geometry_msgs interactive_markers kdl_conversions kdl_parser message_generation roscpp sensor_msgs std_msgs std_srvs tf_conversions tf visualization_msgs)

find_package(orocos_kdl REQUIRED)

### Message Generation ###
generate_dynamic_reconfigure_options(cfg/FrameTracker.cfg)

add_action_files(
  DIRECTORY action
  FILES FrameTracking.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cob_frame_tracker
  CATKIN_DEPENDS actionlib_msgs cob_srvs control_toolbox dynamic_reconfigure geometry_msgs interactive_markers kdl_conversions kdl_parser message_runtime roscpp rospy sensor_msgs std_msgs std_srvs tf_conversions tf visualization_msgs
)

### BUILD ###
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(cob_frame_tracker src/cob_frame_tracker.cpp)
add_dependencies(cob_frame_tracker cob_srvs_gencpp)
add_dependencies(cob_frame_tracker ${cob_frame_tracker_EXPORTED_TARGETS})

add_library(interactive_frame_target src/interactive_frame_target.cpp)
add_dependencies(interactive_frame_target cob_srvs_gencpp)

add_executable(cob_frame_tracker_node src/cob_frame_tracker_node.cpp)
target_link_libraries(cob_frame_tracker_node cob_frame_tracker ${catkin_LIBRARIES})

add_executable(interactive_frame_target_node src/interactive_frame_target_node.cpp)
target_link_libraries(interactive_frame_target_node interactive_frame_target ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS cob_frame_tracker cob_frame_tracker_node interactive_frame_target interactive_frame_target_node
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
 PATTERN ".svn" EXCLUDE
)

install(PROGRAMS scripts/interactive_frame_target.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
