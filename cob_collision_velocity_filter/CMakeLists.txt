cmake_minimum_required(VERSION 2.8.3)
project(cob_collision_velocity_filter)

find_package(catkin REQUIRED COMPONENTS
  cob_footprint_observer
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  roscpp
  tf
  tf2_ros
  visualization_msgs
)

find_package(Boost REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX)
  add_definitions(-std=gnu++0x)
else()
  add_definitions(-std=c++0x)
endif()

### Dynamic reconfigure ###
generate_dynamic_reconfigure_options(cfg/CollisionVelocityFilter.cfg)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs visualization_msgs
)

### BUILD ###
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(collision_velocity_filter src/${PROJECT_NAME}.cpp src/velocity_limited_marker.cpp)
add_dependencies(collision_velocity_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(collision_velocity_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})

### INSTALL ###
install(TARGETS collision_velocity_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
