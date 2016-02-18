cmake_minimum_required(VERSION 2.8.3)
project(cob_omni_drive_controller)

find_package(catkin REQUIRED COMPONENTS angles controller_interface geometry_msgs hardware_interface nav_msgs sensor_msgs std_srvs tf urdf)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  CATKIN_DEPENDS angles controller_interface geometry_msgs hardware_interface nav_msgs sensor_msgs std_srvs tf urdf
  DEPENDS Boost
  INCLUDE_DIRS include
  LIBRARIES cob_omni_drive_geom cob_omni_drive_controller
)

### BUILD ###
include_directories(include src ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(cob_omni_drive_geom src/UndercarriageCtrlGeom.cpp src/param_parser.cpp)
add_dependencies(cob_omni_drive_geom ${catkin_EXPORTED_TARGETS})
target_link_libraries(cob_omni_drive_geom ${catkin_LIBRARIES})

add_library(cob_omni_drive_controller src/odom_plugin.cpp src/control_plugin.cpp)
add_dependencies(cob_omni_drive_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(cob_omni_drive_controller cob_omni_drive_geom ${catkin_LIBRARIES} ${Boost_LIBRARIES})

### INSTALL ###
install(TARGETS cob_omni_drive_geom cob_omni_drive_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
