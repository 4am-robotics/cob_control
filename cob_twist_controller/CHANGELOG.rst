^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_twist_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
