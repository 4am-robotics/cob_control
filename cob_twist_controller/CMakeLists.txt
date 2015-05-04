cmake_minimum_required(VERSION 2.8.3)
project(cob_twist_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp sensor_msgs geometry_msgs nav_msgs kdl_parser kdl_conversions tf tf_conversions dynamic_reconfigure)

find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

find_package(orocos_kdl REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/TwistController.cfg
)

# Declare catkin package
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES cob_twist_controller augmented_solver damping_methods constraint_solvers limiters
   CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} )


## Build
set(SRC_C_DIR "src/constraint_solvers")
set(SRC_CS_DIR "${SRC_C_DIR}/solvers")

# Declare a cpp library
add_library(damping_methods src/damping_methods/damping_base.cpp src/damping_methods/damping.cpp)
add_library(constraint_solvers ${SRC_CS_DIR}/constraint_solver_base.cpp ${SRC_CS_DIR}/unconstraint_solver.cpp ${SRC_CS_DIR}/wln_joint_limit_avoidance_solver.cpp ${SRC_CS_DIR}/weighted_least_norm_solver.cpp ${SRC_C_DIR}/constraint_solver_factory_builder.cpp)
add_library(limiters src/limiters/limiter_base.cpp src/limiters/limiter.cpp)
target_link_libraries(constraint_solvers damping_methods)

add_library(augmented_solver src/augmented_solver.cpp)
target_link_libraries(augmented_solver constraint_solvers ${orocos_kdl_LIBRARIES})

add_library(cob_twist_controller src/cob_twist_controller.cpp )
add_dependencies(augmented_solver cob_twist_controller_gencfg)
target_link_libraries(cob_twist_controller augmented_solver limiters ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

## Declare a cpp executable
add_executable(cob_twist_controller_node src/cob_twist_controller_node.cpp)
target_link_libraries(cob_twist_controller_node cob_twist_controller ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

## Install
# Mark executables and/or libraries for installation
install(TARGETS cob_twist_controller cob_twist_controller_node augmented_solver constraint_solvers damping_methods limiters
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/cob_twist_controller/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
 PATTERN ".svn" EXCLUDE
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(PROGRAMS scripts/test_publisher_twist.py scripts/test_publisher_twist_stamped.py scripts/test_publisher_twist_series.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

