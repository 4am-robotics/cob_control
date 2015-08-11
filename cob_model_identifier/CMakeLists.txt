cmake_minimum_required(VERSION 2.8.3)
project(cob_model_identifier)

find_package(catkin REQUIRED COMPONENTS cmake_modules geometry_msgs kdl_parser roscpp sensor_msgs std_srvs tf)

find_package(Boost REQUIRED)

find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

find_package(TinyXML REQUIRED)

catkin_package(
  CATKIN_DEPENDS geometry_msgs kdl_parser roscpp sensor_msgs std_srvs tf
  DEPENDS Boost TinyXML
  INCLUDE_DIRS include
  LIBRARIES input_generator output_recorder
)

### BUILD ###
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})

add_library(output_recorder src/output_recorder.cpp)
add_dependencies(output_recorder ${catkin_EXPORTED_TARGETS})
target_link_libraries(output_recorder ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${orocos_kdl_INCLUDE_DIRS})

add_executable(output_recorder_node src/output_recorder_node.cpp)
target_link_libraries(output_recorder_node output_recorder ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS output_recorder_node output_recorder
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(PROGRAMS scripts/input_series_twist.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

install(DIRECTORY launch
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
