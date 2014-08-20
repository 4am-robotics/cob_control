cmake_minimum_required(VERSION 2.8.3)
project(cob_hardware_interface)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS hardware_interface controller_manager brics_actuator )

find_package(Boost REQUIRED COMPONENTS system thread)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS 
  INCLUDE_DIRS include
  LIBRARIES cob_hwinterface_vel
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )


## Build
# Declare a cpp library
add_library(cob_hwinterface_vel src/cob_hwinterface_vel.cpp )
add_dependencies(cob_hwinterface_vel brics_actuator_gencpp)

## Declare a cpp executable
add_executable(cob_hwinterface_vel_node src/cob_hwinterface_vel_node.cpp)
target_link_libraries(cob_hwinterface_vel_node cob_hwinterface_vel ${catkin_LIBRARIES} ${Boost_LIBRARIES} )


## Install
# Mark executables and/or libraries for installation
install(TARGETS cob_hwinterface_vel cob_hwinterface_vel_node
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

#install(DIRECTORY launch
 #DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
