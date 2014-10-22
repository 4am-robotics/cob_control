cmake_minimum_required(VERSION 2.8.3)
project(cob_model_identifier)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs cmake_modules tf kdl_parser kdl_conversions std_srvs cob_srvs)

find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})


# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES input_generator output_recorder
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs tf std_srvs cob_srvs
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} )


## Build
# Declare a cpp library
add_library(input_generator src/input_generator.cpp )
add_library(output_recorder src/output_recorder.cpp )

## Declare a cpp executable
add_executable(input_generator_node src/input_generator_node.cpp)
target_link_libraries(input_generator_node input_generator ${catkin_LIBRARIES})

add_executable(output_recorder_node src/output_recorder_node.cpp)
target_link_libraries(output_recorder_node output_recorder ${catkin_LIBRARIES} ${orocos_kdl_INCLUDE_DIRS} )


## Install
# Mark executables and/or libraries for installation
install(TARGETS input_generator_node input_generator output_recorder_node output_recorder
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

install(DIRECTORY launch config
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
