cmake_minimum_required(VERSION 2.8.3)
project(cob_twist_controller)

find_package(catkin REQUIRED COMPONENTS cmake_modules cob_obstacle_distance cob_srvs dynamic_reconfigure eigen_conversions geometry_msgs kdl_conversions kdl_parser nav_msgs roscpp sensor_msgs std_msgs tf urdf visualization_msgs)

find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

find_package(orocos_kdl REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/TwistController.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES damping_methods inv_calculations constraint_solvers limiters hardware_interfaces kinematic_extensions inverse_differential_kinematics_solver twist_controller
  CATKIN_DEPENDS cob_frame_tracker cob_obstacle_distance cob_srvs dynamic_reconfigure eigen_conversions geometry_msgs kdl_conversions kdl_parser nav_msgs robot_state_publisher roscpp rospy rviz sensor_msgs std_msgs tf topic_tools urdf visualization_msgs xacro
)

### BUILD ###
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})

set(SRC_C_DIR "src/constraint_solvers")
set(SRC_CS_DIR "${SRC_C_DIR}/solvers")

add_library(damping_methods src/damping_methods/damping.cpp)
add_library(inv_calculations src/inverse_jacobian_calculations/inverse_jacobian_calculation.cpp)
add_library(constraint_solvers ${SRC_CS_DIR}/unconstraint_solver.cpp ${SRC_CS_DIR}/wln_joint_limit_avoidance_solver.cpp ${SRC_CS_DIR}/weighted_least_norm_solver.cpp ${SRC_CS_DIR}/gradient_projection_method_solver.cpp ${SRC_CS_DIR}/stack_of_tasks_solver.cpp ${SRC_CS_DIR}/stack_of_tasks_gpm_solver.cpp ${SRC_CS_DIR}/stack_of_tasks_gpm_solver.cpp ${SRC_CS_DIR}/task_priority_solver.cpp ${SRC_CS_DIR}/dynamic_tasks_readjust_solver.cpp ${SRC_C_DIR}/constraint_solver_factory.cpp)
add_dependencies(constraint_solvers cob_srvs_gencpp)
target_link_libraries(constraint_solvers damping_methods inv_calculations)

add_library(limiters src/limiters/limiter.cpp)
target_link_libraries(limiters ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_library(hardware_interfaces src/hardware_interface_types/hardware_interface_type.cpp)
target_link_libraries(hardware_interfaces ${catkin_LIBRARIES})

add_library(kinematic_extensions src/kinematic_extensions/kinematic_extension.cpp)
target_link_libraries(kinematic_extensions ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_library(inverse_differential_kinematics_solver src/inverse_differential_kinematics_solver.cpp src/callback_data_mediator.cpp)
target_link_libraries(inverse_differential_kinematics_solver constraint_solvers kinematic_extensions ${orocos_kdl_LIBRARIES})

add_library(twist_controller src/cob_twist_controller.cpp)
add_dependencies(inverse_differential_kinematics_solver cob_twist_controller_gencfg)
target_link_libraries(twist_controller inverse_differential_kinematics_solver limiters hardware_interfaces ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(cob_twist_controller_node src/cob_twist_controller_node.cpp)
target_link_libraries(cob_twist_controller_node twist_controller ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(debug_trajectory_marker_node src/debug/debug_trajectory_marker_node.cpp)
target_link_libraries(debug_trajectory_marker_node ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS cob_twist_controller_node damping_methods inv_calculations constraint_solvers limiters hardware_interfaces kinematic_extensions inverse_differential_kinematics_solver twist_controller debug_trajectory_marker_node
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/cob_twist_controller/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
 PATTERN ".svn" EXCLUDE
)

install(PROGRAMS scripts/test_publisher_twist.py scripts/test_publisher_twist_stamped.py scripts/test_publisher_twist_series.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
