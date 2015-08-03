cmake_minimum_required(VERSION 2.8.3)
project(cob_model_identifier)

find_package(catkin REQUIRED COMPONENTS cmake_modules geometry_msgs kdl_parser roscpp sensor_msgs std_msgs std_srvs tf)

find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES input_generator output_recorder
  CATKIN_DEPENDS geometry_msgs kdl_parser  roscpp rospy sensor_msgs std_msgs std_srvs tf
)

### BUILD ###
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})

add_library(output_recorder src/output_recorder.cpp)
target_link_libraries(output_recorder ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${orocos_kdl_INCLUDE_DIRS})

add_executable(output_recorder_node src/output_recorder_node.cpp)
target_link_libraries(output_recorder_node output_recorder)

### INSTALL ###
install(TARGETS output_recorder_node output_recorder
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
 PATTERN ".svn" EXCLUDE
)

install(PROGRAMS scripts/input_series_twist.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

install(DIRECTORY launch
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
