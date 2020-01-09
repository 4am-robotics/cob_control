cmake_minimum_required(VERSION 2.8.3)
project(cob_obstacle_distance)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS cob_control_msgs cob_srvs dynamic_reconfigure eigen_conversions geometry_msgs kdl_conversions kdl_parser moveit_msgs roscpp roslib roslint sensor_msgs shape_msgs std_msgs tf tf_conversions urdf visualization_msgs)

find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(Eigen3)
set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN3_DEFINITIONS})

find_package(orocos_kdl REQUIRED)

find_package(PkgConfig REQUIRED)

pkg_check_modules(ASSIMP assimp)

pkg_check_modules(LIBFCL REQUIRED fcl)
find_library(LIBFCL_LIBRARIES_FULL ${LIBFCL_LIBRARIES} ${LIBFCL_LIBRARY_DIRS})
set(fcl_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")

catkin_package(
  CATKIN_DEPENDS cob_control_msgs cob_srvs dynamic_reconfigure eigen_conversions geometry_msgs kdl_conversions kdl_parser moveit_msgs roscpp roslib sensor_msgs shape_msgs std_msgs tf tf_conversions urdf visualization_msgs
  DEPENDS Boost
  INCLUDE_DIRS include
  LIBRARIES parsers marker_shapes_management
)

### BUILD ###
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${FCL_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${ASSIMP_INCLUDE_DIRS})

add_library(parsers src/parsers/mesh_parser.cpp src/parsers/stl_parser.cpp)
add_dependencies(parsers ${catkin_EXPORTED_TARGETS})
target_link_libraries(parsers assimp ${fcl_LIBRARIES} ${catkin_LIBRARIES})

add_library(marker_shapes_management  src/link_to_collision.cpp src/marker_shapes/marker_shapes_impl.cpp src/marker_shapes/marker_shapes_interface.cpp src/shapes_manager.cpp)
add_dependencies(marker_shapes_management ${catkin_EXPORTED_TARGETS})
target_link_libraries(marker_shapes_management parsers ${fcl_LIBRARIES} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(${PROJECT_NAME} src/chainfk_solvers/advanced_chainfksolver_recursive.cpp src/${PROJECT_NAME}.cpp src/distance_manager.cpp src/helpers/helper_functions.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} parsers marker_shapes_management ${fcl_LIBRARIES} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(debug_obstacle_distance_node src/debug/debug_obstacle_distance_node.cpp)
add_dependencies(debug_obstacle_distance_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(debug_obstacle_distance_node ${catkin_LIBRARIES})

roslint_cpp()

### Install ###
install(TARGETS ${PROJECT_NAME} debug_obstacle_distance_node marker_shapes_management parsers
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/example_obstacle_publisher_node.py scripts/test_interactive_obstacle_node.py scripts/test_obstacle_publisher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

install(DIRECTORY config launch
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
