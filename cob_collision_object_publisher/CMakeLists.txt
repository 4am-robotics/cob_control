cmake_minimum_required(VERSION 2.8.3)
project(cob_collision_object_publisher)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Load catkin and all dependencies required for this package
#find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp sensor_msgs geometry_msgs nav_msgs kdl_parser kdl_conversions tf tf_conversions dynamic_reconfigure)
find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp sensor_msgs geometry_msgs nav_msgs tf tf_conversions dynamic_reconfigure message_generation)

find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

# find_package(orocos_kdl REQUIRED)
find_package(fcl REQUIRED)

#generate_dynamic_reconfigure_options(
#  cfg/TwistController.cfg
#)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ObjectOfInterest.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# Declare catkin package
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES cob_collision_object_publisher
   CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${FCL_INCLUDE_DIRS})


## Build

# Declare a cpp library
#add_library(augmented_solver src/augmented_solver.cpp)
#target_link_libraries(augmented_solver constraint_solvers chainfk_solvers ${orocos_kdl_LIBRARIES})

#add_library(cob_twist_controller src/cob_twist_controller.cpp )
#add_dependencies(augmented_solver cob_twist_controller_gencfg)
#target_link_libraries(cob_twist_controller augmented_solver limiters ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

## Declare a cpp executable
add_executable(cob_collision_object_publisher src/cob_collision_object_publisher.cpp)
target_link_libraries(cob_collision_object_publisher ${fcl_LIBRARIES} ${catkin_LIBRARIES})

## Install
# Mark executables and/or libraries for installation
# install(TARGETS cob_twist_controller cob_twist_controller_node augmented_solver constraint_solvers damping_methods limiters inv_calculations chainfk_solvers
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

# Mark other files for installation (e.g. launch and bag files, etc.)
#install(PROGRAMS scripts/test_publisher_twist.py scripts/test_publisher_twist_stamped.py scripts/test_publisher_twist_series.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
#)

