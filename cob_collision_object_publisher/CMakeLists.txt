cmake_minimum_required(VERSION 2.8.3)
project(cob_collision_object_publisher)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp sensor_msgs geometry_msgs nav_msgs tf tf_conversions dynamic_reconfigure message_generation)

find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

# find_package(orocos_kdl REQUIRED)
find_package(fcl REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ObjectOfInterest.srv
)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#)

# Declare catkin package
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES 
   CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${FCL_INCLUDE_DIRS})


## Build

## Declare a cpp executable
add_executable(cob_collision_object_publisher src/cob_collision_object_publisher.cpp src/marker_shapes_interface.cpp src/shapes_manager.cpp src/distance_manager.cpp)
target_link_libraries(cob_collision_object_publisher ${fcl_LIBRARIES} ${catkin_LIBRARIES})

## Install
# Mark executables and/or libraries for installation
install(TARGETS cob_collision_object_publisher
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/cob_collision_object_publisher/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
 PATTERN ".svn" EXCLUDE
)

