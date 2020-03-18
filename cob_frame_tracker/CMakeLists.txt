cmake_minimum_required(VERSION 2.8.3)
project(cob_frame_tracker)

find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs cob_srvs control_toolbox dynamic_reconfigure geometry_msgs interactive_markers kdl_conversions kdl_parser message_generation roscpp roslint sensor_msgs std_msgs std_srvs tf visualization_msgs)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(orocos_kdl REQUIRED)

### Message Generation ###
generate_dynamic_reconfigure_options(cfg/FrameTracker.cfg)

add_action_files(
  DIRECTORY action
  FILES FrameTracking.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs cob_srvs control_toolbox dynamic_reconfigure geometry_msgs interactive_markers kdl_parser message_runtime roscpp sensor_msgs std_msgs std_srvs tf visualization_msgs
  DEPENDS Boost
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} interactive_frame_target
)

### BUILD ###
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(interactive_frame_target src/interactive_frame_target.cpp)
add_dependencies(interactive_frame_target ${catkin_EXPORTED_TARGETS})
target_link_libraries(interactive_frame_target ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(interactive_frame_target_node src/interactive_frame_target_node.cpp)
target_link_libraries(interactive_frame_target_node interactive_frame_target ${catkin_LIBRARIES})

add_executable(spacenav_commander src/spacenav_commander.cpp)
add_dependencies(spacenav_commander ${catkin_EXPORTED_TARGETS})
target_link_libraries(spacenav_commander ${catkin_LIBRARIES})

roslint_cpp()

### INSTALL ###
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node interactive_frame_target interactive_frame_target_node spacenav_commander
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/interactive_frame_target.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
