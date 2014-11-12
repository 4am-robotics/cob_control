cmake_minimum_required(VERSION 2.8.3)
project(cob_twist_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp sensor_msgs geometry_msgs brics_actuator kdl_parser kdl_conversions tf tf_conversions dynamic_reconfigure)

find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

find_package(orocos_kdl REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/TwistController.cfg
)

# Declare catkin package
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES cob_twist_controller augmented_solver output_publisher
   CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs brics_actuator
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} )


## Build
# Declare a cpp library
add_library(augmented_solver src/augmented_solver.cpp)
target_link_libraries(augmented_solver ${orocos_kdl_LIBRARIES})

add_library(${PROJECT_NAME} src/cob_twist_controller.cpp )
add_dependencies(${PROJECT_NAME} brics_actuator_gencpp)
add_dependencies(augmented_solver ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} augmented_solver ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_library(output_publisher src/output_publisher.cpp )

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node src/cob_twist_controller_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(output_publisher_node src/output_publisher_node.cpp)
target_link_libraries(output_publisher_node output_publisher ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})


## Install
# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node augmented_solver output_publisher output_publisher_node
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
