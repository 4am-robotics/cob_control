^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_twist_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.20 (2022-11-17)
-------------------

0.8.19 (2022-07-29)
-------------------

0.8.18 (2022-01-12)
-------------------

0.8.17 (2021-12-23)
-------------------
* Merge pull request `#262 <https://github.com/ipa320/cob_control/issues/262>`_ from fmessmer/debug_cart_vel_recursive
  publish cart vel magnitudes for frames of chain
* fix catkin_lint
* calculate fk_vel_recursive based on joint_trajectory_controller state for both desired and actual joint states
* rename debug node
* fix getNrOfJoints vs getNrOfSegments and fixed joint handling
* debug chain
* provide launch file for debug_evaluate_jointstates
* fill header.stamp in twist_magnitude msg
* extend debug_evaluate_jointstates_node to recursively publish cart vel magnitudes for frames of chain
* Contributors: Felix Messmer, fmessmer

0.8.16 (2021-10-19)
-------------------

0.8.15 (2021-05-17)
-------------------

0.8.14 (2021-05-10)
-------------------
* Merge pull request `#254 <https://github.com/ipa320/cob_control/issues/254>`_ from mikaelarguedas/python3-six
  ROS_PYTHON_VERSION conditional dependency for python-six
* mark exec_depend
* ROS_PYTHON_VERSION conditional dependency for python-six
* Contributors: Felix Messmer, Mikael Arguedas, fmessmer

0.8.13 (2021-04-06)
-------------------

0.8.12 (2020-10-21)
-------------------
* Merge pull request `#243 <https://github.com/ipa320/cob_control/issues/243>`_ from fmessmer/test_noetic
  test noetic
* ROS_PYTHON_VERSION conditional dependency for matplotlib
* conditional depend for orocos-kdl
* use setuptools instead of distutils
* Bump CMake version to avoid CMP0048 warning
* Contributors: Felix Messmer, fmessmer

0.8.11 (2020-03-21)
-------------------

0.8.10 (2020-03-18)
-------------------
* Merge pull request `#228 <https://github.com/ipa320/cob_control/issues/228>`_ from fmessmer/feature/python3_compatibility_melodic
  [ci_updates] pylint + Python3 compatibility - melodic
* disable simple_script_server import error
* Use six.moves.input for all uses of raw_input/input
* fix modules
* fix pylint errors
* python3 compatibility via 2to3
* Merge pull request `#226 <https://github.com/ipa320/cob_control/issues/226>`_ from fmessmer/ci_updates_melodic
  [travis] ci updates - melodic
* catkin_lint fixes
* Contributors: Felix Messmer, Loy van Beek, fmessmer

0.8.1 (2019-11-07)
------------------

0.8.0 (2019-08-09)
------------------

0.7.8 (2019-08-09)
------------------
* Merge pull request `#210 <https://github.com/ipa320/cob_control/issues/210>`_ from fmessmer/missing_target_dependency
  add missing target dependency
* add missing target dependency
* Contributors: Felix Messmer, fmessmer

0.7.7 (2019-08-06)
------------------

0.7.6 (2019-06-07)
------------------

0.7.5 (2019-05-20)
------------------
* Merge pull request `#206 <https://github.com/ipa320/cob_control/issues/206>`_ from fmessmer/improve_lookat_extension
  improve lookat extension
* implement lookat_pointing_frame
* fix lookat_chain offset, allow KinematicExtension to fail, param-config converter functions
* Contributors: Felix Messmer, fmessmer

0.7.4 (2019-04-05)
------------------

0.7.3 (2019-03-14)
------------------
* Merge pull request `#197 <https://github.com/ipa320/cob_control/issues/197>`_ from ipa320/fmessmer-patch-1
  add missing install tags
* add missing install tags
* Contributors: Felix Messmer

0.7.2 (2018-07-21)
------------------
* update maintainer
* Contributors: fmessmer

0.7.1 (2018-01-07)
------------------
* Merge remote-tracking branch 'origin/kinetic_release_candidate' into kinetic_dev
* Merge pull request `#169 <https://github.com/ipa320/cob_control/issues/169>`_ from ipa-fxm/kinetic_updates_indigo
  Kinetic updates indigo
* Merge branch 'indigo_dev' of github.com:ipa320/cob_control into kinetic_dev
  Conflicts:
  .travis.yml
* Merge pull request `#159 <https://github.com/ipa320/cob_control/issues/159>`_ from ipa-fxm/APACHE_license
  use license apache 2.0
* use license apache 2.0
* Contributors: Felix Messmer, ipa-fxm, ipa-uhr-mk

0.7.0 (2017-07-18)
------------------
* Merge branch 'indigo_dev' of https://github.com/ipa320/cob_control into kinetic_dev
* [kinetic] migration (`#115 <https://github.com/ipa320/cob_control/issues/115>`_)
  * [kinetic] find package Eigen3 instead of Eigen
  * [kinetic] switched from fcl to libfcl-dev dependency
  * ignore cob_obstacle_distance for now
  * [kinetic] use industrial_ci for travis
  * [kinetic] use industrial_cis ipa320 fork & notify on_success only on change
  * fixed fcl dependency
  * new fcl version switched from boost::shared to std::shared
  * whitelist package
  * use ros-industrials fork
  * cleaned up travis.yml
* Contributors: Benjamin Maidel, ipa-fxm

0.6.15 (2017-07-18)
-------------------
* handle continuous joints in ControllerInterfaceJointStates
* handle continuous joints in position_limiter
* harmonize createTask
* remove unused TaskDamping
* remove ConstraintParamFactory
* harmonize ConstraintParams
* set default values for damping methods
* own param struct for unified solver
* New singularity and joint limit avoidance method (`#149 <https://github.com/ipa320/cob_control/issues/149>`_)
  * first implementation of new jla solver using sigmoid functions
  * first version of new JLA method working
  * last fix to new JLA solver
  * correction in parameter used in sigmoid damping method
  * new solver added and new jla method added to be tested and selected according to performance comparison
  * fix to rqt_reconfigure parameters
  * final correction to new solver and new weighting method
  * prints removed
  * square trajectory added
  * wln with sigmoid solver corrected
  * final corrections
  * final corrections to the solver formula
  * created unified singularity and joint limit avoidance class... other classes moved
  * restoring wln previous version of the solver
  * saturation block added in unified solver just for testing
  * saturation block corrected
  * checking new solver and constraints
  * corrected new test script for the new method
  * removed test files and corrected identation
  * defining GPM solver as default
  * fix identation
  * comment corrected
  * identation fix in contraint solver factory
  * finalize PR
* cleanup leftovers
* pluginlib approach for controller interfaces
* add toggle for enforce_input_limits, implement both All and Individual Cartesian limiting
* combine LimiterXContainer
* limiter fixed
* Limiter Base class removed
* pull request changes
* final implementation of cartesian limiter
* bug fixed in class limiter container
* bug fixed in class limiter container
* cartesian limits introduced
* remove sympy implementation and dependency
* use xacro --inorder
* final roslint
* applied changes request in the pull request review
* damping factor for old method replaced by a diagonal matrix
* last corretions to pull request
* corrections added according to last Felix comments
* fixed changes done by rebase
* prints removed for new damping function calculations
* new method based of sigmoid function to singularity avoidance implemented
* bug fix in least square calculation when changing to return matrix
* starting to roslint some packages
* manually fix changelog
* Contributors: Bruno Brito, ipa-fxm

0.6.14 (2016-10-10)
-------------------

0.6.13 (2016-10-10)
-------------------

0.6.12 (2016-10-10)
-------------------
* cob_twist_controller: fix add_dependencies calls
* Contributors: ipa-mig

0.6.11 (2016-04-01)
-------------------
* reduce output
* outsource obstacle_distance messages
* remove support for interpol_position controller
* debug output
* use joint_group_velocity_controller for torso extension
* verify position limit scaling factor
* fix typo
* fix lookat: do not look backwards
* missing sympy dependency
* add test publisher twist sine
* avoid unecessary service calls to obstacle_distance
* fix collision avoidance dimension segfault
* infinitesiamal threshold for BaseActive
* wider limits
* enforce position limit
* add test_forward_command_sine_node
* cleanup period
* Update test_trajectory_command_sine_node.cpp
* adjust lookat extension limits
* more compact parameter structure
* make lookat linear axis configurable - axis and offset
* cleanup roslint
* add trajectory_command test node
* introduce cfg-parameter integrator_smoothing
* add debug publisher to simpson_integration
* rename member variables
* add q_dot_ik smoothing, adjust parameters
* add timing members for period
* Merge pull request `#79 <https://github.com/ipa320/cob_control/issues/79>`_ from ipa-fxm/fix_visualize_twist_marker
  visualize twist marker
* proper reset for ControllerInterfaceJointStates
* visualize angular twist
* fix visualize twist marker
* Fixed the TwistDirection Marker
* use undamped jacobian in nullspace projection
* minor renaming
* roslint cob_twist_controller
* final roslint
* add TimeStamp to trajectory_interface
* also print limiting joint
* prepare remapping for twist_mux in cartesian controller
* add test nodes for SimpsonIntegrator
* reset moving average
* simplify API
* use new API in SimpsonIntegrator
* test new MovingAverage API
* all new MovingAverage API
* saver initialization of weighting
* test scripts for moving_average
* use interpolated position controller
* add more debug scripts
* consider various roslint/styleguide hints
* apply change in parameter name
* fix frame_id in visualizeTwist
* lookat extension fully implemented
* more experiments with reset condition in simpson_integrator
* minor improvement of comment
* fix order of doxygen comment
* fix whitespaces
* move simpson integration to new util class
* prepare structure for lookat
* temporarily disable CA when being used together with KinematicExtensions
* less output
* more consistent code structure for constraints
* remove obsolete return values
* use extension_ratio for all extensions
* wip: consider kinematic_extensions within limiters and constraints - still unstable
* chain not needed in limiters
* resolve hardcoded cycle time in prediction
* proper generation of Jacobian for kinematic extension from urdf
* more generic naming in extension_dof, transform extension_jacobian in extension_urdf
* merge with demo updates
* fix BASE_COMPENSATION
* Fixed order of transform and service registration. Additionally added more time to wait for service availability.
* fix dimension of jac_extension
* merge and roslint
* roslint cob_twist_controller
* draft towards kinematic_extension for COB_TORSO based on URDF
* prepare structure for additional kinematic_extensions
* revert acceleration_limiters impl, class structure only, further consistency changes and cleanup
* progress with acceleration limiters, still wip
* implement acceleration limiter
* pass down whole JointStates structure
* better reset condition
* fix limiter reset, fix service existence, consistency
* prepare structure for acceleration limiters
* temporary cleanup
* do FK_Vel in GPM for debugging
* further debug gpm and self-motion
* add solveTwist duration output
* visualization marker for desired twist direction
* renaming: hardware_interface to controller_interface
* fixes for positional interfaces
* waitForExistence of registerLink service
* wip: use undamped inverse in gpm
* allow to calculate un-damped, un-truncated inverse jacobian
* update octave scripts for testing variants
* working on marker publisher
* working on log output
* update trajectory_hardware_interface
* introduce HardwareInterfacePositionBase, reset Integration on out-dated data
* handle base_compensation in kinematic_extension enum
* renaming frame - link
* Merge branch 'indigo_dev' of github.com:ipa-fxm/cob_control into trajectory_hardware_interface
* - Commented output lines. - Renamed frame_of_interest to link_of_interest.
* Merge branch 'indigo_dev' of github.com:ipa-fxm/cob_control into trajectory_hardware_interface
* implement JointTrajectory hardware_interface for twist_controller
* Contributors: Felix Messmer, Marco Bezzon, ipa-fxm, ipa-fxm-cm, ipa-fxm-mb

0.6.10 (2015-08-31)
-------------------

0.6.9 (2015-08-25)
------------------
* authors in package xml
* Corrections integrated from PR: [WIP] Finalizing/Testing of TwistController features (`#51 <https://github.com/ipa-fxm/cob_control/issues/51>`_). Renaming from frame_to_collision to link_to_collision.
* - Fixed bug in constraint implementation: sign was "-" but must be "+".
  - Moved scripts.
* - Added more text to package.xml
  - Moved scripts to subfolder test.
* Added new script for raw3-1.
* In config file avoided setting of 0 tolerance (DIV/0!). Removed additional output.
* - Made some changes for test. - Decreased Duration time for markers. - Corrected pose update for self collision check frames. - Added new scripts.
* Added comment to activation buffer.
* boost revision
* Merge branch 'test_of_feature' into test_of_feature_with_adapt_frame_tracker
* Overwritten numerical_filtering with false.
* Some preparations for test: IMarker smaller, Alpha settings, More scripts. Default value for Frame Tracker params.
* - Added parameters for activation buffer and critical threshold of CA and JLA constraints.
  - Removed method getActivationThreshold because parameter can be used directly.
  - Packed thresholds into struct.
  - Commented some outputs.
* - CA: Increased exp. decay from 0.1 to 0.2 - Added comments. - Moved constraints set and management to base class. - Added time delta to test script.
* - Removed parameter mu. - Added obstacle id for identification of collision pair in ObstacleDistance.msg. - Added Frametracking to DataCollector. - Restructured obstacle distance data collecting. - In debug trajectory marker added explicit usage of frame_tracker/tracking_frame.
* merge
* explicit dependency to boost
* Fixed bug when obstacles move away from robot. Clear distances list when new distances arrive (also in case nothing is available for current link) to avoid no movement.
* - Renaming obstacle_marker_server for interactive obstacle
  - Now publishing all obstacle distances instead of the minimal distance only. Defined a MIN_DISTANCE for selection of data to publish (e.g. > than 0.5 m doesn't make sense for CA).
  - Selection of the minimal distance in debug node.
  - Callback data mediator processes all obstacles for a frame of interest id now.
  - Restructured methods in constraint classes.
  - In CA constraint now processing all collision pairs for one link in a CollisionAvoidance instance.
  - Removed unnecessary output.
* Resolved merge conflicts.
* more fixes for migration afer merge
* merge with package_xml_2
* remove trailing whitespaces
* migrate to package format 2
* - Removed unnecessary commented code.
* - Added handling of no exception: Save files.
* - Avoided drawing of self-collision frames -> can be done via rviz.
  - Increased CA activation threshold to 0.25 m
* Deleted unnecessary files.
* - Renamed dynamics_tasks_readjust_solver -> stack_of_tasks_solver. Therefore adapted corresponding cfg and data_types.
  - Created Python package for data collection.
* fix HardwareInterfacePosition
* consider PR review comments
* - Removed experiment solvers for task stacks. Now the dynamic_tasks_readjust_solver works better than them.
  - adapted MakeLists and config and data_types.
* - For BVH introduced a shared_ptr member -> so a collision object can be created without copying the whole BVH. This saves computation time (5% for 3 SCA and 1 torus)
  - Decreased rate for cob_obstacle_distance because the movement does not change that often.
  - According to the rate adapted the moving average for distance in constraint_ca_impl
* updates from ipa-fxm-mb
* Implemented Python package to set dyn_reconfigure params. Made test_move_around_torus use of this class.
* cleanup
* sort dependencies
* Created test, Removed commends, Removed output.
* Fixed bugs in cartesian_controller: waitFor last available transform else extrapolation error; send always a new constructed StampedTransform instead of using an already existent one, else end-effector is decoupled from manipulator and other confusing things happen...; Added responsible node to tf error msg.
* Merged with ipa-fxm/test_of_feature branch.
* Separated JLA and CA constraints from constraint_impl.h
* - Corrected JLA constraint. - Added weighting of GPM prio dependent. - Added buffer region for CA constraint to become active.
* Made movinge average generic for other data types. Using moving average for CA constraint.
* simplify simpson
* - Removed PredictDistance Service (not necessary anymore; found a lightweight computational algorithm).
  - Made KDL::ChainFkSolverVel_recursive in CA constraint available for prediction.
  - Replace constraints update method prediction variable with JntArrayVel.
  - Refactored ObstacleDistance.msg: Reduced number of members, renamings, added frame_of_interest for registration and made use of header->frame_id for arm_base_link.
  - Renamed service for registration.
  - Improved input twist damping in case of a constraint is in CRITICAL state.
* check for frame existence
* allow target_frame to be configured via private param, beautifying
* Further tests and adaptations for test.
* Added generated const from .cfg; Styling
* re-implementation of trajectory_publisher in c++
* Added Python package to collect data and write collected data into a file.
* Fixed parameter initialization.
* fix parameter initialization + add max_vel_base to cfg
* add topic name to ROS_WARN output
* add doxygen documentation
* add example launch file
* publish joint_states in separate thread
* adding JointStateInterface
* add base_marker to publisher
* generalize scripts, minor changes
* Corrected default values in cfg.
* Corrected CMakeLists.txt. Replaced ASSIMP_LIBRARIES with assimp.
* Added consideration of origin from URDF tags. Removed shape_type and so Registration.srv and replaced by SetString service. Removed comments.
* Considering visual tag as fallback now. Removed duplicate map and struct.
* Considered further proposals from https://github.com/ipa-fxm/cob_control/pull/7.
* Considered proposals from https://github.com/ipa-fxm/cob_control/pull/7
* Integrated comments of https://github.com/ipa-fxm/cob_control/pull/7. Replaced static link2collision map with URDF parser. Added class for URDF parser and create marker shapes.
* Added functions to represent a registered robot link as a mesh instead of simple shapes. Added a mapping between robot link name and mesh resource name.
* Integration. To avoid controller jump into critical region again introduced in cart vel damping.
* Fixed DIV/0 error in distance cost function calculation.
* Reassignment of corrected values to twist_controller_params\_ instance.
* Added JLA inequality constraint to be used within the dynamic task strategy. Added checking and resetting of dynamic_reconfigure params. Corrected formatting of LSV damping.
* Moved TaskStackController to parameters list. Added new damping factor for constraints (to avoid algo. singularities). Added new inverse for testing.
* Separated constraints from solvers and vice versa. Added new parameters. Prettified GUI.
* Added a Simple Python node to publish a line strip to see the real trajectory and the desired one.
* Made CA possible with active base. Bug fixing of solvers in case of base active. Corrected JLA constraints.
* more style unification
* parameter initialization
* enforceLimits now in inv_diff_kin_solver
* enum for KinematicExtension and styling for constants
* consider remarks from CodeReview: mainly styling and beautify
* hardware_interface_type renaming
* re-arrange Parameter structs
* Merge branch 'task_stack_prio_feature' of github.com:ipa-fxm-mb/cob_control into multi_feature_merge
* Added new method for dynamic tasks readjustment. Implemented prediction of distance now for vectors.
* resolve conflicts after merging ipa-fxm-mb/task_stack_prio_feature
* KinematicExtensionBaseActive works
* WIP: further cleanup and introduction of abstract helper class
* WIP: kinematic_extension replaces base_active
* Refactored task stack solvers. Fixed creation of solver instances. Removed unnecessary test code.
* beautify and code-review
* remove auto generatable doc
* merge with ipa320
* generic interface types
* Added chain recursive fk vel calculator. Corrected calculation of translational Jacobian for CA. Introduced further msg types to achieve that. Extended solvers: CA as first prio task, CA as GPM, CA as GPM with disappearing main tasks.
* Added task stack controller.
* Corrected dist calclation for GPM CA
* Added stack of tasks and Macijewski task prio CA.
* Added stack of tasks and further developments on GPM CA.
* Further developments.
* Implemented proposals from discussion https://github.com/ipa320/cob_control/pull/38. Removed tabs. Corrected node handles.
* Merge with IPA320 Indigo Dev.
* removed bug
* merged
* Added moving average filter and simpson integration formula
* New octave script to check whether split of vector v into separate tasks works.
* Removed rad variable.
* Fixed issue in WLN_JLA: Removed conversion to radian.
* Made code more CppStyleGuide ROS compliant.
* Made corrections proposed in https://github.com/ipa320/cob_control/pull/38#
* - Renaming: AugmentedSolver -> InverseDifferentialKinematicsSolver
  - Merged cob_twist_controller_data_types and augmented_solver_data_types -> cob_twist_controller_data_types
  - Renamings: According to ROS C++ Style Guide.
* - Added doxygen comments
  - Corrected the messages produced by catkin_lint
  - Created a static method to return SolverFactory
* - Made obstacle tracking independent from arm_right.
  - Refactored signatures of solve methods: Instead of using dynamic vector now a 6d vector is used because twists are of dim 6d.
  - Removed unnecessary comments.
  - Introduced eigen_conversions to have simple converters instead of filling matrices and vectors manually -> Reduces typing and copying errors!
* - Renamed some variables according to ROS C++ style guide
  - Moved advanced chain fk solver from cob_twist_controller to cob_obstacle_distance.
  - Replaced complicated transformation of base_link to arm_base_link with simpler and direct one.
  - Removed unnecessary services and replaced with message publisher and subscriber (for distance calculation).
  - Added example launch file for cob_obstacle_distance.
  - Corrected handling of objects of interest. Now in both packages frames are used (instead of joint names) -> made it similar to KDL and tf handlings.
  - Removed commented code.
  - Removed pointer where objects could be used directly (constraint params generation)
  - callback data mediator keeps old distance values until new ones were received. An iterator is used to go through the container.
* Added missing modules
* - Created a obstacle distance publisher in cob_obstacle_distance package and a subscriber in cob_twist_controller package.
  - Created registration service in cob_obstacle_distance
  - Creation of multiple CA constraints dependent on formerly registered joint regions.
* test
* Renaming
* Added collision avoidance feature. Solve with GPM. Made usage of cob_collision_object_publisher via ROS service.
* Added possibility to calculate self motion magnitude dependent from joint velocity limits.
* Removed tracking error publisher / subscriber and removed additional p gain for PD-Control (already done in FrameTracker with PID controller)
* Solved merge conflicts
* WIP:
  - Added new solver feature: GradientProjectionMethod.
  - Added cost function for: JLA, JLA_MID, CA
  - Added kappa parameter to set GPM scaling.
  - Added builder to support build of multiple constraints.
* Added new implementation for KDL::ChainFkSolverPos_recursive. Provides storage of joint positions.
* Beautify.
  Corresponding to PR https://github.com/ipa-fxm-mb/cob_control/pull/1.
* renamed parameters and functions
* Generischer Ansatz
* Low Isotropic Damping
* - Added constraints for JLA and JLA mid.
  - Added calculation for step size.
* - Prepared the implementation of a builder to create a set of constraints.
  - Decoupled constraints generation from solver class GPM (now they could be used for other methods as well).
  - Removed asParams from constraints. Only necessary for constraintParams.
* - Added a possibility to implement constraint functions.
  - Added a registration mechanism to the solver (registration in a priorized set).
  - Added a parameter to select it
* - Renamed pseudoinverse_calculation -> inverse_jacobian_calculation
* - Decoupled pseudoinverse calculation from constraint_solvers. That allows new implementations for pseudoinverse calculations. Additionally it allows to calculate pseudoinverses of further Jacobians (e.g. for constraints)
  - Removed unnecessary _base.cpp files and removed them from CMakeLists.txt.
* - Refactored parametrization of damping -> damping method is now given to solver for extensions (like numerical filtering)
  - Considered damping method NONE in case of no damping for solving IK.
* - Added a publisher for the tracking errors to send them to cob_twist_controller
  - Added a subscriber to collect the errors and put them to the solver.
  - Added a parameter to set the p gain. If 0.0 old behavior is active (default value).
* Contributors: ipa-fxm, ipa-fxm-cm, ipa-fxm-mb

* add missing include
* Contributors: ipa-fxm

* missing dependency
* Contributors: ipa-fxm

0.6.8 (2015-06-17)
------------------

0.6.7 (2015-06-17)
------------------
* reduce output in limiters
* restructure namespaces for parameters of cartesian controllers
* - Instead of creating png create eps.
* - Added new damping method None
  - Added enum value to select damping None
  - Removed pure pointer usage and added boost::shared_ptr usage (which provides pointer management / ensure deletion of objects)
  - Removed unused includes
  - Renamings
* - Removed unnecessary ROS_INFO_STREAMs
  - Removed temporary variables for test code
* - Added debug code
  - Removed truncation
  - Removed unused members
* - Grouped limiters in one .h and one .cpp
  - Grouped damping_methods in one .h and one .cpp
  - Removed separate factories. Made SolverFactory generic by introducing template parameters.
  - Made usage of boost::shared_ptr instead of own pointer handling.
  - Adapted CMakeLists.txt according to changes.
  - Split parameter enforce_limits into enforce_pos_limits and enforce_vel_limits
* - To enforce limits for joint positions and velocities created new classes.
  - Additionally added parameter for keeping direction or not when enforcing limits.
  - Therefore removed normalize_velocities and enforce_limits from cob_twist_controller. Instead the new limiter_container is used.
  - Added new struct to provide cob_twist_controller params.
  - Removed debug code.
* - Take care: W^(1/2) * q_dot = weighted_pinv_J * x_dot -> One must consider the weighting!!!
  - Added script to check pseudo-inverse calculation.
* - Take care: W^(1/2) * q_dot = weighted_pinv_J * x_dot -> One must consider the weighting!!!
  - Added an octave script to verify the statement above.
* - Removed unnecessary file
* - Added doxygen comments
  - Activated graphviz for doc generation
  - Added const to method signatures to avoid undesired JntArray-Data change.
* - moved enfore_limits from augmented_solver to cob_twist_controller
  - Added a base case WeightedLeastNorm to constraint solvers
  Instantiated it acts like an unconstraint solver.
  - Renamed JointLimitAvoidanceSolver to WLN_JointLimitAvoidanceSolver
  - WLN_JointLimitAvoidanceSolver inherits from WeightedLeastNormSolver and implements calculate_weighting
* - moved enfore_limits from augmented_solver to cob_twist_controller
  - Added a base case WeightedLeastNorm to constraint solvers
  Instantiated it acts like an unconstraint solver.
  - Renamed JointLimitAvoidanceSolver to WLN_JointLimitAvoidanceSolver
  - WLN_JointLimitAvoidanceSolver inherits from WeightedLeastNormSolver and implements calculate_weighting only. -> Solving is done by the WLN Solver.
* Added validation outputs.
  Added comments for doxygen generation.
  Did some renaming.
* Made restructured changes active.
  Corrected some implementation.
  Activated both old and new implementation for comparison and testing purposes.
* Made usages of ConstraintSolverFactoryBuilder:
  - Creates DampingMethod
  - Creates ConstraintSolver
  - Executes calculation of joint velocities.
* Split up augmented_solver.cpp into different constraint solvers: JLA constraints and unconstraint.
* - Restructured augmented_solver.
  - Renamed class augmented_solver to AugmentedSolver.
  - Created damping_methods as classes to ease creation of dampings (and new ones).
* add comments
* cleanup
* beautify CMakeLists
* using correct base topic names
* fix debug node
* remove obsolete code for parameter initialization, enforce_limits behaviour
* revision, simplification and cleanup
* remove obsolete files
* twist controller analyser
* last update
* update working frame_tracker
* base compensation test
* temporary adjust base topics
* reduce output
* twist series test script
* use component specific joint_states topic
* no output
* merge
* cleaning up
* new publisher and transformation names
* merge with cm
* added commentary, tolerance as dynamic reconfigure, modified enforce_limits
* Debug functions
* merge with cm
* Merge branch 'indigo_dev' of https://github.com/ipa320/cob_control into fm_cm_merged_new
* last commit before merging
* new rqt_features
* delete all test packages
* delete all test packages
* fixed errors from merging
* merged from ipa-fxm-cm
* beautify, added commentary, limit enforcing and dynamic reconfigure for JLA
* new debug twist
* add tracking_action
* test
* new features
* test
* Merge branch 'merge_fm_cm' of github.com:ipa-fxm-fm/cob_control into cm_dev
* changes
* Corrected errors from merging
* First merge attempt
* Joint Limit Avoidance added and cleaned up
* Added publisher for the pose
* Modified for the new structure
* cleaned up again
* Cleaned up
* New features
* a commit a day keeps the doctor away
* Contributors: Christian Ehrmann, ipa-fxm, ipa-fxm-cm, ipa-fxm-fm, ipa-fxm-mb

0.6.6 (2014-12-18)
------------------
* remove dep to cob_srvs and std_srvs
* Contributors: Florian Weisshardt

0.6.5 (2014-12-18)
------------------
* Merge branch 'indigo_dev' into indigo_release_candidate
* add dep
* Contributors: Florian Weisshardt

0.6.4 (2014-12-16)
------------------

0.6.3 (2014-12-16)
------------------
* add dependency to nav_msgs
* Contributors: Florian Weisshardt

0.6.2 (2014-12-15)
------------------
* Merge branch 'indigo_dev' into indigo_release_candidate
* fix twist_control dimensions for any-DoF
* merge with fxm-cm
* merge with fxm-fm
* cleaning up
* branch with features for merging
* topics according to new structure
* remove brics_actuator
* more topic renaming according to new structure
* renaming debug topic
* adapt namespaces for cartesian_controller to new structure
* dynamic reconfigure
* revision of cob_twist_controller
* merge_cm
* merge_fm
* temporary commit
* temporary commit
* changes in initialization
* restructure test_twist publisher scripts
* fix twist_controller to be usable without base again
* able to add base DoFs to Jacobian solver - first tests - needs more debugging
* null-space syncMM
* add test script for twist_stamped
* able to apply twists wrt to various coordinate system orientations
* cleanup, restructure and fix
* missing include
* merge with fxm-fm + clean up
* add twist publisher script
* add output publisher
* cleaning up
* beautify
* Add fixes provided by @ipa-fxm-fm
* fix controller and add damping
* add twist publisher script
* add output publisher
* Add fixes provided by @ipa-fxm-fm
* Contributors: Florian Weisshardt, ipa-fxm, ipa-fxm-cm, ipa-fxm-fm

0.6.1 (2014-09-22)
------------------

0.5.4 (2014-08-26)
------------------
* fix dependency-hell on multiple cores
* moved cob_twist_controller
* Contributors: Alexander Bubeck, ipa-fxm
