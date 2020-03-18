cmake_minimum_required(VERSION 2.8.3)
project(cob_base_velocity_smoother)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure geometry_msgs nav_msgs roscpp roslint std_msgs)

find_package(Boost REQUIRED thread)

generate_dynamic_reconfigure_options(cfg/params.cfg)

if(CMAKE_COMPILER_IS_GNUCXX)
  add_definitions(-std=gnu++0x)
else()
  add_definitions(-std=c++0x)
endif()

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs std_msgs
)

### BUILD ###
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${BOOST_LIBRARIES})

add_executable(velocity_smoother src/${PROJECT_NAME}/velocity_smoother.cpp src/velocity_smoother_node.cpp)
add_dependencies(velocity_smoother ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocity_smoother ${catkin_LIBRARIES} ${BOOST_LIBRARIES})

roslint_cpp()

### INSTALL ###
install(TARGETS ${PROJECT_NAME} velocity_smoother
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
