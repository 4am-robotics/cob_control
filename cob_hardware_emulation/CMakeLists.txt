cmake_minimum_required(VERSION 2.8.3)
project(cob_hardware_emulation)

find_package(catkin REQUIRED COMPONENTS roscpp rosgraph_msgs)

catkin_package(
  CATKIN_DEPENDS rosgraph_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(emulation_clock src/emulation_clock.cpp)
add_dependencies(emulation_clock ${catkin_EXPORTED_TARGETS})
target_link_libraries(emulation_clock ${catkin_LIBRARIES})

install(TARGETS emulation_clock
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/emulation_base.py
  scripts/emulation_clock.py
  scripts/emulation_follow_joint_trajectory.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
