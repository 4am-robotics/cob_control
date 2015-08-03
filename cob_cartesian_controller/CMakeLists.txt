cmake_minimum_required(VERSION 2.8.3)
project(cob_cartesian_controller)

find_package(catkin REQUIRED COMPONENTS cmake_modules cob_srvs geometry_msgs kdl_conversions roscpp std_msgs std_srvs tf visualization_msgs)

find_package(TinyXML REQUIRED)
find_package(orocos_kdl REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cartesian_controller
  CATKIN_DEPENDS cob_srvs geometry_msgs kdl_conversions roscpp rospy std_msgs std_srvs tf visualization_msgs
)

### BUILD ###
include_directories(include ${catkin_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})

add_library(cartesian_controller src/cartesian_controller.cpp)
add_dependencies(cartesian_controller cob_srvs_gencpp)
target_link_libraries(cartesian_controller ${catkin_LIBRARIES} ${TinyXML_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES})

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
 FILES_MATCHING PATTERN "*.h"
 PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch movement
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
