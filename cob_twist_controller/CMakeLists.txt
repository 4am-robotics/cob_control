cmake_minimum_required(VERSION 3.0.2)
project(cob_twist_controller)

find_package(catkin REQUIRED COMPONENTS cmake_modules cob_control_msgs cob_srvs control_msgs dynamic_reconfigure eigen_conversions geometry_msgs kdl_conversions kdl_parser nav_msgs pluginlib roscpp roslint sensor_msgs std_msgs tf tf_conversions trajectory_msgs urdf visualization_msgs)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

find_package(orocos_kdl REQUIRED)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/TwistController.cfg
)

catkin_package(
  CATKIN_DEPENDS cob_control_msgs cob_srvs control_msgs dynamic_reconfigure eigen_conversions geometry_msgs kdl_conversions kdl_parser nav_msgs pluginlib roscpp sensor_msgs std_msgs tf tf_conversions trajectory_msgs urdf visualization_msgs
  DEPENDS Boost
  INCLUDE_DIRS include
  LIBRARIES damping_methods inv_calculations constraint_solvers limiters controller_interfaces kinematic_extensions inverse_differential_kinematics_solver twist_controller
)

### BUILD ###
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})

set(SRC_C_DIR "src/constraint_solvers")
set(SRC_CS_DIR "${SRC_C_DIR}/solvers")

## plugin library
add_library(controller_interfaces src/controller_interfaces/controller_interface.cpp)
add_dependencies(controller_interfaces ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller_interfaces ${catkin_LIBRARIES})

## internal libraries
add_library(damping_methods src/damping_methods/damping.cpp)
add_dependencies(damping_methods ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(damping_methods ${catkin_LIBRARIES})

add_library(inv_calculations src/inverse_jacobian_calculations/inverse_jacobian_calculation.cpp)
add_dependencies(inv_calculations ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(inv_calculations ${catkin_LIBRARIES})

add_library(constraint_solvers ${SRC_C_DIR}/constraint_solver_factory.cpp ${SRC_CS_DIR}/gradient_projection_method_solver.cpp ${SRC_CS_DIR}/stack_of_tasks_solver.cpp ${SRC_CS_DIR}/task_priority_solver.cpp ${SRC_CS_DIR}/unconstraint_solver.cpp ${SRC_CS_DIR}/unified_joint_limit_singularity_solver.cpp ${SRC_CS_DIR}/weighted_least_norm_solver.cpp ${SRC_CS_DIR}/wln_joint_limit_avoidance_solver.cpp)
add_dependencies(constraint_solvers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(constraint_solvers damping_methods inv_calculations)

add_library(limiters src/limiters/limiter.cpp)
add_dependencies(limiters ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(limiters ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_library(kinematic_extensions src/kinematic_extensions/kinematic_extension_builder.cpp src/kinematic_extensions/kinematic_extension_dof.cpp src/kinematic_extensions/kinematic_extension_lookat.cpp src/kinematic_extensions/kinematic_extension_urdf.cpp)
add_dependencies(kinematic_extensions ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinematic_extensions ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_library(inverse_differential_kinematics_solver src/callback_data_mediator.cpp src/inverse_differential_kinematics_solver.cpp)
add_dependencies(inverse_differential_kinematics_solver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(inverse_differential_kinematics_solver constraint_solvers kinematic_extensions ${orocos_kdl_LIBRARIES})

add_library(twist_controller src/${PROJECT_NAME}.cpp)
add_dependencies(twist_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(twist_controller inverse_differential_kinematics_solver limiters ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node twist_controller ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})


### DEBUG NODES ###
add_executable(debug_trajectory_marker_node src/debug/debug_trajectory_marker_node.cpp)
add_dependencies(debug_trajectory_marker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(debug_trajectory_marker_node ${catkin_LIBRARIES})

add_executable(debug_fk_vel_recursive_node src/debug/debug_fk_vel_recursive_node.cpp)
add_dependencies(debug_fk_vel_recursive_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(debug_fk_vel_recursive_node ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(test_moving_average_node src/debug/test_moving_average_node.cpp)
add_dependencies(test_moving_average_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_moving_average_node ${catkin_LIBRARIES})

add_executable(test_simpson_integrator_node src/debug/test_simpson_integrator_node.cpp)
add_dependencies(test_simpson_integrator_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_simpson_integrator_node ${catkin_LIBRARIES})

add_executable(test_trajectory_command_sine_node src/debug/test_trajectory_command_sine_node.cpp)
add_dependencies(test_trajectory_command_sine_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_trajectory_command_sine_node ${catkin_LIBRARIES})

add_executable(test_forward_command_sine_node src/debug/test_forward_command_sine_node.cpp)
add_dependencies(test_forward_command_sine_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_forward_command_sine_node ${catkin_LIBRARIES})

add_executable(test_twist_command_sine_node src/debug/test_twist_command_sine_node.cpp)
add_dependencies(test_twist_command_sine_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_twist_command_sine_node ${catkin_LIBRARIES})

roslint_cpp()

### INSTALL ###
install(TARGETS ${PROJECT_NAME}_node constraint_solvers controller_interfaces damping_methods inv_calculations inverse_differential_kinematics_solver kinematic_extensions limiters twist_controller
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS debug_fk_vel_recursive_node debug_trajectory_marker_node test_moving_average_node test_simpson_integrator_node test_trajectory_command_sine_node test_twist_command_sine_node
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/test_publisher_float_sine_noise.py scripts/test_publisher_twist_sine.py scripts/test_publisher_twist.py scripts/test_publisher_twist_stamped.py scripts/test_publisher_twist_series.py scripts/test_publisher_vel.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/collect_twist_control_eval_data.py scripts/evaluate_dbg_jnt_velocity_tests.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/test/test_careobot_base.py scripts/test/test_careobot_GPM_jla_ca_torus.py scripts/test/test_careobot_st_jla_ca_selfcollision_arm_left_direct_head.py scripts/test/test_careobot_st_jla_ca_selfcollision_arm_left_internal_motion.py scripts/test/test_careobot_st_jla_ca_sphere.py scripts/test/test_careobot_st_jla_ca_torus.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES controller_interface_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config files launch
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
