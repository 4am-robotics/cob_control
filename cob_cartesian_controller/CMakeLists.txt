cmake_minimum_required(VERSION 2.8.3)
project(cob_cartesian_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    cmake_modules 
    std_msgs 
    geometry_msgs 
    visualization_msgs 
    std_srvs 
    cob_srvs 
    tf 
    actionlib_msgs
    message_generation
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES CartesianController.action
)

add_message_files(
  DIRECTORY msg
  FILES MoveLin.msg
        MoveCirc.msg
        Profile.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs actionlib_msgs
)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES profile_generator trajectory_interpolator cartesian_controller cartesian_controller_utils
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs visualization_msgs std_srvs cob_srvs tf actionlib_msgs message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Build
# Declare a cpp library
add_library(cartesian_controller_utils src/cartesian_controller_utils.cpp)
target_link_libraries(cartesian_controller_utils ${catkin_LIBRARIES})

add_library(profile_generator src/trajectory_profile_generator/trajectory_profile_generator_lin.cpp src/trajectory_profile_generator/trajectory_profile_generator_circ.cpp)
add_dependencies(profile_generator ${PROJECT_NAME}_gencpp)
target_link_libraries(profile_generator ${catkin_LIBRARIES})

add_library(trajectory_interpolator src/trajectory_interpolator/trajectory_interpolator.cpp)
target_link_libraries(trajectory_interpolator profile_generator ${catkin_LIBRARIES})

add_library(cartesian_controller src/cartesian_controller.cpp)
target_link_libraries(cartesian_controller trajectory_interpolator cartesian_controller_utils ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Declare a cpp executable
add_executable(cartesian_controller_node src/cartesian_controller_node.cpp)
target_link_libraries(cartesian_controller_node cartesian_controller ${catkin_LIBRARIES})

## Install
# Mark executables and/or libraries for installation
install(TARGETS profile_generator trajectory_interpolator cartesian_controller cartesian_controller_utils cartesian_controller_node
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
 PATTERN ".svn" EXCLUDE
)

install(PROGRAMS scripts/test_move_lin.py scripts/test_move_circ.py scripts/test_move_santa.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

install(DIRECTORY config launch
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
