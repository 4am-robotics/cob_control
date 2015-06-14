cmake_minimum_required(VERSION 2.8.3)
project(cob_cartesian_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp cmake_modules std_msgs geometry_msgs visualization_msgs std_srvs cob_srvs tf kdl_conversions roslib)

find_package(TinyXML REQUIRED)
find_package(orocos_kdl REQUIRED)


# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cartesian_controller
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs visualization_msgs std_srvs cob_srvs tf kdl_conversions
)

include_directories(include ${catkin_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})

## Build
# Declare a cpp library
add_library(cartesian_controller src/cartesian_controller.cpp )
add_dependencies(cartesian_controller cob_srvs_gencpp)

## Declare a cpp executable
add_executable(cartesian_controller_node src/cartesian_controller_node.cpp)
target_link_libraries(cartesian_controller_node cartesian_controller  ${catkin_LIBRARIES} ${TinyXML_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES})

## Install
# Mark executables and/or libraries for installation
install(TARGETS cartesian_controller cartesian_controller_node
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

install(DIRECTORY launch movement
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
