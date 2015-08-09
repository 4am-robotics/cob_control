cmake_minimum_required(VERSION 2.8.3)
project(cob_cartesian_controller)

find_package(catkin REQUIRED COMPONENTS cmake_modules cob_srvs geometry_msgs roscpp roslib std_msgs std_srvs tf visualization_msgs)

catkin_package(
  CATKIN_DEPENDS cob_srvs geometry_msgs roscpp roslib std_msgs std_srvs tf visualization_msgs
  INCLUDE_DIRS include
  LIBRARIES cartesian_controller
)

### BUILD ###
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(cartesian_controller src/cartesian_controller.cpp)
add_dependencies(cartesian_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(cartesian_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(cartesian_controller_node src/cartesian_controller_node.cpp)
target_link_libraries(cartesian_controller_node cartesian_controller ${catkin_LIBRARIES})

### INSTALL ##
install(TARGETS cartesian_controller cartesian_controller_node
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch movement
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
