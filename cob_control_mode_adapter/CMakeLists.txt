cmake_minimum_required(VERSION 2.8.3)
project(cob_control_mode_adapter)

find_package(catkin REQUIRED COMPONENTS controller_manager_msgs roscpp roslint std_msgs)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package()

### BUILD ###
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(cob_control_mode_adapter_node src/cob_control_mode_adapter_node.cpp)
add_dependencies(cob_control_mode_adapter_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(cob_control_mode_adapter_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

roslint_cpp()

### INSTALL ###
install(TARGETS cob_control_mode_adapter_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
