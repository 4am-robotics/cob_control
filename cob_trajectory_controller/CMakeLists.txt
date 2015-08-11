cmake_minimum_required(VERSION 2.8.3)
project(cob_trajectory_controller)

find_package(catkin REQUIRED COMPONENTS actionlib cmake_modules cob_srvs control_msgs dynamic_reconfigure roscpp sensor_msgs std_msgs std_srvs trajectory_msgs)

find_package(orocos_kdl REQUIRED)

### Message Generation ###
generate_dynamic_reconfigure_options(
  cfg/CobTrajectoryController.cfg
)

catkin_package(
  CATKIN_DEPENDS actionlib cob_srvs control_msgs dynamic_reconfigure roscpp sensor_msgs std_msgs std_srvs trajectory_msgs
  DEPENDS orocos_kdl
  INCLUDE_DIRS common/include
)

### BUILD ###
include_directories(common/include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ros/src/${PROJECT_NAME}.cpp common/src/genericArmCtrl.cpp common/src/RefValJS_PTP.cpp common/src/RefValJS_PTP_Trajectory.cpp common/src/TimeStamp.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

### INSTALL ###
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
