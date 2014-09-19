cmake_minimum_required(VERSION 2.8.3)
project(cob_lookat_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs brics_actuator std_srvs kdl_parser kdl_conversions tf tf_conversions )

find_package(orocos_kdl REQUIRED)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs brics_actuator std_srvs
  INCLUDE_DIRS include
  LIBRARIES cob_lookat_driver cob_lookat_controller
)

include_directories(include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})


## Build
# Declare a cpp library
add_library(cob_lookat_driver src/cob_lookat_driver.cpp )
add_dependencies(cob_lookat_driver brics_actuator_gencpp)

add_library(cob_lookat_controller src/cob_lookat_controller.cpp )
add_dependencies(cob_lookat_controller brics_actuator_gencpp)
target_link_libraries(cob_lookat_controller ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

## Declare a cpp executable
add_executable(cob_lookat_driver_node src/cob_lookat_driver_node.cpp)
target_link_libraries(cob_lookat_driver_node cob_lookat_driver ${catkin_LIBRARIES})

add_executable(cob_lookat_controller_node src/cob_lookat_controller_node.cpp)
target_link_libraries(cob_lookat_controller_node cob_lookat_controller ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})


## Install
# Mark executables and/or libraries for installation
install(TARGETS cob_lookat_driver cob_lookat_driver_node cob_lookat_controller cob_lookat_controller_node
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
