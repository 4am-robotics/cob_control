cmake_minimum_required(VERSION 2.8.3)
project(cob_undercarriage_ctrl_node)

find_package(catkin REQUIRED COMPONENTS cob_msgs cob_omni_drive_controller control_msgs diagnostic_msgs diagnostic_updater geometry_msgs nav_msgs roscpp tf urdf)

catkin_package()

###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_new src/cob_undercarriage_ctrl_new.cpp)
add_dependencies(${PROJECT_NAME}_new ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_new ${catkin_LIBRARIES})

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}_new
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
