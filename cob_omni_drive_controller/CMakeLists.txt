cmake_minimum_required(VERSION 3.0.2)
project(cob_omni_drive_controller)

find_package(catkin REQUIRED COMPONENTS angles cob_base_controller_utils controller_interface dynamic_reconfigure geometry_msgs hardware_interface nav_msgs pluginlib realtime_tools roscpp sensor_msgs std_msgs std_srvs tf tf2 urdf)

find_package(Boost REQUIRED COMPONENTS thread)

generate_dynamic_reconfigure_options(
  cfg/SteerCtrl.cfg
)

catkin_package(
  CATKIN_DEPENDS angles cob_base_controller_utils controller_interface dynamic_reconfigure geometry_msgs hardware_interface nav_msgs roscpp sensor_msgs std_msgs std_srvs tf tf2 urdf
  DEPENDS Boost
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} cob_omni_drive_geom
)

### BUILD ###
include_directories(include src ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(cob_omni_drive_geom src/UndercarriageCtrlGeom.cpp src/param_parser.cpp)
add_dependencies(cob_omni_drive_geom ${catkin_EXPORTED_TARGETS})
target_link_libraries(cob_omni_drive_geom ${catkin_LIBRARIES})

add_library(${PROJECT_NAME} src/control_multi_plugin.cpp src/control_plugin.cpp src/odom_plugin.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} cob_omni_drive_geom ${catkin_LIBRARIES} ${Boost_LIBRARIES})

### INSTALL ###
install(TARGETS ${PROJECT_NAME} cob_omni_drive_geom
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
