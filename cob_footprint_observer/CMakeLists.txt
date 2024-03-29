cmake_minimum_required(VERSION 3.0.2)
project(cob_footprint_observer)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation roscpp std_msgs tf)

find_package(Boost REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX)
  add_definitions(-std=gnu++0x)
else()
  add_definitions(-std=c++0x)
endif()

### Message Generation ###
add_service_files(
  FILES
  GetFootprint.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime roscpp std_msgs tf
)

### BUILD ###
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(footprint_observer src/${PROJECT_NAME}.cpp)
add_dependencies(footprint_observer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(footprint_observer ${catkin_LIBRARIES} ${Boost_LIBRARIES})

### INSTALL ###
install(TARGETS footprint_observer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
