cmake_minimum_required(VERSION 2.8.3)
project(cob_obstacle_distance)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp roslib shape_msgs moveit_msgs sensor_msgs geometry_msgs visualization_msgs tf tf_conversions dynamic_reconfigure message_generation kdl_parser kdl_conversions eigen_conversions cob_srvs)

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
  PredictDistance.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

# Declare catkin package
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES parsers marker_shapes_management
   CATKIN_DEPENDS roscpp shape_msgs moveit_msgs sensor_msgs geometry_msgs visualization_msgs message_runtime cob_srvs
   DEPENDS assimp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${FCL_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${ASSIMP_INCLUDE_DIRS})

## Build
add_library(parsers src/parsers/stl_parser.cpp src/parsers/mesh_parser.cpp)
target_link_libraries(parsers assimp ${fcl_LIBRARIES} ${catkin_LIBRARIES})

add_library(marker_shapes_management src/marker_shapes/marker_shapes_impl.cpp src/marker_shapes/marker_shapes_interface.cpp src/shapes_manager.cpp src/frame_to_collision.cpp) 
target_link_libraries(marker_shapes_management parsers ${fcl_LIBRARIES} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

## Declare a cpp executable
add_executable(cob_obstacle_distance src/cob_obstacle_distance.cpp src/helpers/helper_functions.cpp src/distance_manager.cpp src/chainfk_solvers/advanced_chainfksolver_recursive.cpp)
add_dependencies(cob_obstacle_distance cob_srvs_gencpp)
target_link_libraries(cob_obstacle_distance parsers marker_shapes_management ${fcl_LIBRARIES} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

## Install
# Mark executables and/or libraries for installation
install(TARGETS cob_obstacle_distance parsers marker_shapes_management
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

install(PROGRAMS scripts/example_obstacle_publisher_node.py scripts/test_obstacle_publisher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

install(DIRECTORY config launch
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

