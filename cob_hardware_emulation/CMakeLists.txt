cmake_minimum_required(VERSION 3.0.2)
project(cob_hardware_emulation)

find_package(catkin REQUIRED COMPONENTS roscpp rosgraph_msgs)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  CATKIN_DEPENDS roscpp rosgraph_msgs
  DEPENDS Boost
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(emulation_clock src/emulation_clock.cpp)
add_dependencies(emulation_clock ${catkin_EXPORTED_TARGETS})
target_link_libraries(emulation_clock ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS emulation_clock
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/emulation_base.py
  scripts/emulation_clock.py
  scripts/emulation_follow_joint_trajectory.py
  scripts/emulation_nav.py
  scripts/emulation_odom_laser.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
