cmake_minimum_required(VERSION 2.8.3)
project(cob_frame_tracker)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED
	COMPONENTS
	roscpp
	std_msgs
	sensor_msgs
	geometry_msgs
	actionlib_msgs
	std_srvs
	cob_srvs
	message_generation
	kdl_parser
	kdl_conversions
	tf
	tf_conversions
	dynamic_reconfigure
	interactive_markers
	control_toolbox)

find_package(orocos_kdl REQUIRED)

generate_dynamic_reconfigure_options(cfg/FrameTracker.cfg)


## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES FrameTracking.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs actionlib_msgs
)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cob_frame_tracker
  CATKIN_DEPENDS
	roscpp
	std_msgs
	sensor_msgs
	geometry_msgs
	actionlib_msgs
	std_srvs
	cob_srvs
	message_runtime
	kdl_parser
	kdl_conversions
	tf
	tf_conversions
	dynamic_reconfigure
	interactive_markers
	control_toolbox
)

include_directories(include ${catkin_INCLUDE_DIRS})


## Build
# Declare a cpp library
add_library(cob_frame_tracker src/cob_frame_tracker.cpp )
add_dependencies(cob_frame_tracker cob_srvs_gencpp)
add_dependencies(cob_frame_tracker ${cob_frame_tracker_EXPORTED_TARGETS})

add_library(interactive_frame_target src/interactive_frame_target.cpp )
add_dependencies(interactive_frame_target cob_srvs_gencpp)

# Declare a cpp executable
add_executable(cob_frame_tracker_node src/cob_frame_tracker_node.cpp)
target_link_libraries(cob_frame_tracker_node cob_frame_tracker ${catkin_LIBRARIES})

add_executable(interactive_frame_target_node src/interactive_frame_target_node.cpp)
target_link_libraries(interactive_frame_target_node interactive_frame_target ${catkin_LIBRARIES})


## Install
# Mark executables and/or libraries for installation
install(TARGETS cob_frame_tracker cob_frame_tracker_node interactive_frame_target interactive_frame_target_node
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
 PATTERN ".svn" EXCLUDE
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(PROGRAMS scripts/interactive_frame_target.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
