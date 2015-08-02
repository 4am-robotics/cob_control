cmake_minimum_required(VERSION 2.8.3)
project(cob_collision_velocity_filter)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs nav_msgs visualization_msgs cob_footprint_observer dynamic_reconfigure tf)

if(CMAKE_COMPILER_IS_GNUCXX)
  add_definitions(-std=gnu++0x)
else()
  add_definitions(-std=c++0x)
endif()

### Dynamic reconfigure ###
generate_dynamic_reconfigure_options(cfg/CollisionVelocityFilter.cfg)

catkin_package(
    CATKIN_DEPENDS roscpp geometry_msgs nav_msgs visualization_msgs cob_footprint_observer dynamic_reconfigure tf
)

### BUILD ###
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(collision_velocity_filter
                        src/cob_collision_velocity_filter.cpp
                        src/velocity_limited_marker.cpp
)

target_link_libraries(collision_velocity_filter ${catkin_LIBRARIES})

add_dependencies(collision_velocity_filter ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(collision_velocity_filter ${catkin}_EXPORTED_TARGETS})


### INSTALL ###
install(TARGETS collision_velocity_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

