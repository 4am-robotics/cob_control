cmake_minimum_required(VERSION 2.8.3)
project(cob_frame_tracker)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs std_srvs cob_srvs tf tf_conversions control_toolbox)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs std_srvs cob_srvs control_toolbox
  INCLUDE_DIRS include
  LIBRARIES cob_frame_tracker
)

include_directories(include ${catkin_INCLUDE_DIRS})


## Build
# Declare a cpp library
add_library(cob_frame_tracker src/cob_frame_tracker.cpp )
add_dependencies(cob_frame_tracker cob_srvs_gencpp)

# Declare a cpp executable
add_executable(cob_frame_tracker_node src/cob_frame_tracker_node.cpp)
target_link_libraries(cob_frame_tracker_node cob_frame_tracker ${catkin_LIBRARIES})


## Install
# Mark executables and/or libraries for installation
install(TARGETS cob_frame_tracker cob_frame_tracker_node
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
