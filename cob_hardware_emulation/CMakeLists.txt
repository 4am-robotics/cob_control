cmake_minimum_required(VERSION 2.8.3)
project(cob_hardware_emulation)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/emulation_base.py
  scripts/emulation_follow_joint_trajectory.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
