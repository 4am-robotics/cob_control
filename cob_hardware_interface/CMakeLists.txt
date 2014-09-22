cmake_minimum_required(VERSION 2.8.3)
project(cob_hardware_interface)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS hardware_interface controller_manager sensor_msgs brics_actuator )

find_package(Boost REQUIRED COMPONENTS system thread)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS hardware_interface controller_manager sensor_msgs brics_actuator
  INCLUDE_DIRS include
  LIBRARIES cob_hwinterface_vel
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )


## Build
# Declare a cpp library
add_library(cob_hwinterface_topics src/cob_hwinterface_topics.cpp )
add_dependencies(cob_hwinterface_topics brics_actuator_gencpp)

## Declare a cpp executable
add_executable(cob_controller_manager_node src/cob_controller_manager_node.cpp)
target_link_libraries(cob_controller_manager_node cob_hwinterface_topics ${catkin_LIBRARIES} ${Boost_LIBRARIES} )


## Install
# Mark executables and/or libraries for installation
install(TARGETS cob_hwinterface_topics cob_controller_manager_node
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

install(DIRECTORY launch
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
