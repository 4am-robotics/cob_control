cmake_minimum_required(VERSION 3.0.2)
project(cob_base_controller_utils)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation nav_msgs roscpp std_msgs std_srvs tf tf2 urdf)

find_package(Boost REQUIRED COMPONENTS thread)

add_message_files(
  DIRECTORY msg
  FILES
  WheelCommands.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime nav_msgs roscpp std_msgs std_srvs tf tf2 urdf
  DEPENDS Boost
  INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(cob_stuck_detector src/stuck_detector.cpp)
add_dependencies(cob_stuck_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cob_stuck_detector ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(cob_spin_detector src/spin_detector.cpp)
add_dependencies(cob_spin_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cob_spin_detector ${catkin_LIBRARIES} ${Boost_LIBRARIES})

### INSTALL ###
install(TARGETS cob_spin_detector cob_stuck_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/test_publisher_twist_sine.py scripts/test_publisher_twist_step.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
