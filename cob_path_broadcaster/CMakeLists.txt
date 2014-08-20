cmake_minimum_required(VERSION 2.8.3)
project(cob_path_broadcaster)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf )

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cob_path_broadcaster
  CATKIN_DEPENDS roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})


## Build
# Declare a cpp library
add_library(cob_path_broadcaster src/cob_path_broadcaster.cpp )

## Declare a cpp executable
add_executable(cob_path_broadcaster_node src/cob_path_broadcaster_node.cpp)
target_link_libraries(cob_path_broadcaster_node cob_path_broadcaster ${catkin_LIBRARIES})


## Install
# Mark executables and/or libraries for installation
install(TARGETS cob_path_broadcaster cob_path_broadcaster_node
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
 PATTERN ".svn" EXCLUDE
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(PROGRAMS scripts/test_cmd_vel_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

#install(DIRECTORY launch config
 #DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
