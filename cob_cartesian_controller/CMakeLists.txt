cmake_minimum_required(VERSION 2.8.3)
project(cob_cartesian_controller)

find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs cob_srvs geometry_msgs message_generation roscpp roslint std_msgs std_srvs tf visualization_msgs)

find_package(Boost REQUIRED)

catkin_python_setup()

### Message Generation ###
add_action_files(
  DIRECTORY action
  FILES CartesianController.action
)

add_message_files(
  DIRECTORY msg
  FILES MoveCirc.msg
        MoveLin.msg
        Profile.msg
)

generate_messages(
  DEPENDENCIES actionlib_msgs geometry_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs cob_srvs geometry_msgs message_runtime roscpp std_msgs std_srvs tf visualization_msgs
  DEPENDS Boost
  INCLUDE_DIRS include
  LIBRARIES profile_generator trajectory_interpolator cartesian_controller cartesian_controller_utils
)

### BUILD ###
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(cartesian_controller_utils src/cartesian_controller_utils.cpp)
add_dependencies(cartesian_controller_utils ${catkin_EXPORTED_TARGETS})
target_link_libraries(cartesian_controller_utils ${catkin_LIBRARIES})

add_library(profile_generator src/trajectory_profile_generator/trajectory_profile_generator_builder.cpp src/trajectory_profile_generator/trajectory_profile_generator_ramp.cpp src/trajectory_profile_generator/trajectory_profile_generator_sinoid.cpp)
add_dependencies(profile_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(profile_generator ${catkin_LIBRARIES})

add_library(trajectory_interpolator src/trajectory_interpolator/trajectory_interpolator.cpp)
add_dependencies(trajectory_interpolator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_interpolator profile_generator ${catkin_LIBRARIES})

add_library(cartesian_controller src/cartesian_controller.cpp)
add_dependencies(cartesian_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cartesian_controller trajectory_interpolator cartesian_controller_utils ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(cartesian_controller_node src/cartesian_controller_node.cpp)
target_link_libraries(cartesian_controller_node cartesian_controller ${catkin_LIBRARIES})

roslint_cpp()

### INSTALL ##
install(TARGETS cartesian_controller cartesian_controller_node cartesian_controller_utils profile_generator trajectory_interpolator
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(
  PROGRAMS scripts/test_move_around_torus.py
           scripts/test_move_circ_interface.py
           scripts/test_move_circ.py
           scripts/test_move_lin_interface.py
           scripts/test_move_lin.py
           scripts/test_move_santa.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
