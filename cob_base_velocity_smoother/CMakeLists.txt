cmake_minimum_required(VERSION 2.8.3)
project(cob_base_velocity_smoother)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp std_msgs)

find_package(Boost REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX)
  add_definitions(-std=gnu++0x)
else()
  add_definitions(-std=c++0x)
endif()

catkin_package()

### BUILD ###
include_directories(include/cob_base_velocity_smoother ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/cob_base_velocity_smoother.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${BOOST_LIBRARIES})

### INSTALL ###
install(TARGETS cob_base_velocity_smoother
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
