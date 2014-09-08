cmake_minimum_required(VERSION 2.8.3)
project(cob_control_mode_adapter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  control_msgs
  controller_manager_msgs
  brics_actuator
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cob_control_mode_adapter
#  CATKIN_DEPENDS brics_actuator control_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(cob_control_mode_adapter
#   src/${PROJECT_NAME}/cob_control_mode_adapter.cpp
# )

# Declare a cpp executable
add_executable(cob_control_mode_adapter_node src/cob_control_mode_adapter_node.cpp)
add_dependencies(cob_control_mode_adapter_node brics_actuator_gencpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(cob_control_mode_adapter_node cob_control_mode_adapter_generate_messages_cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(cob_control_mode_adapter_node
 ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark executables and/or libraries for installation
install(TARGETS cob_control_mode_adapter_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cob_control_mode_adapter.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
