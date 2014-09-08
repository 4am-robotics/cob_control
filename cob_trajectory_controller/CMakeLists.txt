
cmake_minimum_required(VERSION 2.8.3)
project(cob_trajectory_controller)

find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp cob_srvs brics_actuator sensor_msgs std_msgs  actionlib control_msgs dynamic_reconfigure)
find_package(orocos_kdl REQUIRED)
find_package(Eigen REQUIRED)
#######################################
## Declare ROS messages and services ##
#######################################

generate_dynamic_reconfigure_options(
  cfg/CobTrajectoryController.cfg
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
    DEPENDS orocos_kdl
    CATKIN_DEPENDS roscpp std_msgs cob_srvs brics_actuator sensor_msgs actionlib control_msgs dynamic_reconfigure
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

###########
## Build ##
###########

include_directories(common/include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ros/src/${PROJECT_NAME}.cpp common/src/genericArmCtrl.cpp common/src/RefValJS_PTP.cpp  common/src/RefValJS_PTP_Trajectory.cpp common/src/TimeStamp.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_sim ros/src/${PROJECT_NAME}_sim.cpp common/src/genericArmCtrl.cpp common/src/RefValJS_PTP.cpp  common/src/RefValJS_PTP_Trajectory.cpp common/src/TimeStamp.cpp)
add_dependencies(${PROJECT_NAME}_sim ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_sim ${catkin_EXPORTED_TARGETS})

add_executable(cob_simulation_tester ros/src/cob_simulation_tester.cpp)
add_dependencies(cob_simulation_tester ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_sim ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
target_link_libraries(cob_simulation_tester ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})


#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_sim 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
