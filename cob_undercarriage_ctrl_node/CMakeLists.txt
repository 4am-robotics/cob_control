cmake_minimum_required(VERSION 2.8.3)
project(cob_undercarriage_ctrl_node)

# define requierd packages for build this packages/
find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs geometry_msgs diagnostic_msgs tf cob_msgs diagnostic_updater control_msgs cob_omni_drive_controller)

###################################
## catkin specific configuration ##
###################################
# provide lib/packages which are requeird for the use of this package
catkin_package(
    CATKIN_DEPENDS roscpp nav_msgs geometry_msgs diagnostic_msgs tf cob_msgs diagnostic_updater control_msgs cob_omni_drive_controller
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

###########
## Build ##
###########
# add include search paths
include_directories(${catkin_INCLUDE_DIRS})

# add executable for cob_undercarriage_ctrl_node and cob_undercarriage_ctrl_node
add_executable(${PROJECT_NAME}_new src/cob_undercarriage_ctrl_new.cpp)

# link libraries for executable cob_undercarriage_ctrl_node
target_link_libraries(${PROJECT_NAME}_new ${catkin_LIBRARIES} )


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_new
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
