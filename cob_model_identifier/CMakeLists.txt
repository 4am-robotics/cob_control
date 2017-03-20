cmake_minimum_required(VERSION 2.8.3)
project(cob_model_identifier)

find_package(catkin REQUIRED COMPONENTS cmake_modules geometry_msgs kdl_parser roscpp roslint sensor_msgs std_srvs tf)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  set(EIGEN3_DEFINITIONS ${EIGEN_DEFINITIONS})
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
add_definitions(${EIGEN3_DEFINITIONS})

find_package(TinyXML REQUIRED)

catkin_package()

### BUILD ###
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(output_recorder src/output_recorder.cpp)
add_dependencies(output_recorder ${catkin_EXPORTED_TARGETS})
target_link_libraries(output_recorder ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(output_recorder_node src/output_recorder_node.cpp)
target_link_libraries(output_recorder_node output_recorder ${catkin_LIBRARIES})

roslint_cpp()

### INSTALL ###
install(TARGETS output_recorder_node
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/input_series_twist.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

install(DIRECTORY launch
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
