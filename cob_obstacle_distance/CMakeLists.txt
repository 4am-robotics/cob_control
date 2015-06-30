cmake_minimum_required(VERSION 2.8.3)
project(cob_obstacle_distance)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp sensor_msgs geometry_msgs visualization_msgs tf tf_conversions dynamic_reconfigure message_generation kdl_parser kdl_conversions)

find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

find_package(orocos_kdl REQUIRED)

find_package(fcl REQUIRED)

add_message_files(
  FILES
  ObstacleDistance.msg
  ObstacleDistances.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Registration.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

# Declare catkin package
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES
   CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs visualization_msgs message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${FCL_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})

## Build
## Declare a cpp executable
add_executable(cob_obstacle_distance src/cob_obstacle_distance.cpp src/marker_shapes_interface.cpp src/shapes_manager.cpp src/distance_manager.cpp src/chainfk_solvers/advanced_chainfksolverpos_recursive.cpp)
target_link_libraries(cob_obstacle_distance ${fcl_LIBRARIES} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(cob_obstacle_distance cob_obstacle_distance_gencpp)

## Install
# Mark executables and/or libraries for installation
install(TARGETS cob_obstacle_distance
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/cob_obstacle_distance/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h,*.hpp"
 PATTERN ".svn" EXCLUDE
)

