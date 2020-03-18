^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_obstacle_distance
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.10 (2020-03-18)
-------------------
* Merge pull request `#225 <https://github.com/ipa320/cob_control/issues/225>`_ from fmessmer/ci_updates_kinetic
  [travis] ci updates - kinetic
* catkin_lint fixes
* Contributors: Felix Messmer, fmessmer

0.7.9 (2019-11-07)
------------------

0.7.8 (2019-08-09)
------------------

0.7.7 (2019-08-06)
------------------
* Merge pull request `#208 <https://github.com/ipa320/cob_control/issues/208>`_ from ipa-jba/melodic_dev
  [Melodic] melodify
* acommodate kdl api change
* use streams instead of lexical cast (deprecated)
* use urdf ptr types
* Contributors: Felix Messmer, Jannik Abbenseth

0.7.6 (2019-06-07)
------------------

0.7.5 (2019-05-20)
------------------

0.7.4 (2019-04-05)
------------------

0.7.3 (2019-03-14)
------------------

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
* Merge branch 'indigo_release_candidate' into kinetic_release_candidate
* Merge branch 'indigo_dev' of github.com:ipa320/cob_control into multi_distro_travis_kinetic
  Conflicts:
  .travis.yml
  README.md
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
* Added Eigen3 Indigo/Kinetic compatibility
* Contributors: Benjamin Maidel, Denis Štogl, Richard Bormann, ipa-fxm

0.6.15 (2017-07-18)
-------------------
* use xacro --inorder
* Merge pull request `#122 <https://github.com/ipa320/cob_control/issues/122>`_ from ipa-fxm/indigo_roslint
  roslint some packages
* starting to roslint some packages
* manually fix changelog
* add possiblitiy to add shelf as obstacle
* Contributors: Nadia Hammoudeh García, ipa-fxm

0.6.14 (2016-10-10)
-------------------

0.6.13 (2016-10-10)
-------------------

0.6.12 (2016-10-10)
-------------------
* Two publishers where publishing to the same frame name
* Contributors: ipa-bfb

0.6.11 (2016-04-01)
-------------------
* outsource obstacle_distance messages
* add missing install tag
* Added method to set drawable again and to avoid drawing of self-collision links.
* Renamings and replaced box with sphere.
* Fixed order of transform and service registration. Additionally added more time to wait for service availability.
* working on marker publisher
* working on log output
* renaming frame - link
* Contributors: Marco Bezzon, ipa-fxm

0.6.10 (2015-08-31)
-------------------

0.6.9 (2015-08-25)
------------------
* review dependencies
* authors in package xml
* Merge branch 'test_of_feature_with_adapt_frame_tracker' of github.com:ipa-fxm-mb/cob_control into test_of_feature
* Corrections integrated from PR: [WIP] Finalizing/Testing of TwistController features (`#51 <https://github.com/ipa-fxm/cob_control/issues/51>`_). Renaming from frame_to_collision to link_to_collision.
* - Made some changes for test. - Decreased Duration time for markers. - Corrected pose update for self collision check frames. - Added new scripts.
* boost revision
* Some preparations for test: IMarker smaller, Alpha settings, More scripts. Default value for Frame Tracker params.
* - CA: Increased exp. decay from 0.1 to 0.2 - Added comments. - Moved constraints set and management to base class. - Added time delta to test script.
* - Removed parameter mu. - Added obstacle id for identification of collision pair in ObstacleDistance.msg. - Added Frametracking to DataCollector. - Restructured obstacle distance data collecting. - In debug trajectory marker added explicit usage of frame_tracker/tracking_frame.
* merge
* more dependency fixes according to review comments
* explicit dependency to boost
* Fixed bug when obstacles move away from robot. Clear distances list when new distances arrive (also in case nothing is available for current link) to avoid no movement.
* Due to restructuring of self-collision YYAMLs also restructured computation of the file and the ignoreSelfCollisionPart method.
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
* - Avoided drawing of self-collision frames -> can be done via rviz.
  - Increased CA activation threshold to 0.25 m
* - For BVH introduced a shared_ptr member -> so a collision object can be created without copying the whole BVH. This saves computation time (5% for 3 SCA and 1 torus)
  - Decreased rate for cob_obstacle_distance because the movement does not change that often.
  - According to the rate adapted the moving average for distance in constraint_ca_impl
* Merge branch 'indigo_dev' of github.com:ipa320/cob_control into test_of_feature
* Fixed message generation issue
* review dependencies
* updates from ipa-fxm-mb
* updates from ipa320
* cleanup
* missing add_dependencies
* sort dependencies
* Fixed bugs in cartesian_controller: waitFor last available transform else extrapolation error; send always a new constructed StampedTransform instead of using an already existent one, else end-effector is decoupled from manipulator and other confusing things happen...; Added responsible node to tf error msg.
* Merged with ipa-fxm/test_of_feature branch.
* - Corrected JLA constraint. - Added weighting of GPM prio dependent. - Added buffer region for CA constraint to become active.
* - Removed PredictDistance Service (not necessary anymore; found a lightweight computational algorithm).
  - Made KDL::ChainFkSolverVel_recursive in CA constraint available for prediction.
  - Replace constraints update method prediction variable with JntArrayVel.
  - Refactored ObstacleDistance.msg: Reduced number of members, renamings, added frame_of_interest for registration and made use of header->frame_id for arm_base_link.
  - Renamed service for registration.
  - Improved input twist damping in case of a constraint is in CRITICAL state.
* added publisher for path preview
* allow target_frame to be configured via private param, beautifying
* added interactive_obstacle test node, less sleep time on marker publisher
* Further tests and adaptations for test.
* Reduced granularity of a fcl::shape representation. Replaced arm_1_collision with mild.dae.
* Fixed integer size. There might never be 2^64 joints. But maybe more than 255 that's why 2^16 had been chosen.
* finalize example.launch
* move distance_vector marker publisher to separate node
* Reduced granularity of a fcl::shape representation. Replaced arm_1_collision with mild.dae.
* fix sleep rates
* add topic name to ROS_WARN output
* add example launch file
* generalize scripts, minor changes
* Corrected CMakeLists.txt. Replaced ASSIMP_LIBRARIES with assimp.
* Added consideration of origin from URDF tags. Removed shape_type and so Registration.srv and replaced by SetString service. Removed comments.
* Considering visual tag as fallback now. Removed duplicate map and struct.
* Considered further proposals from https://github.com/ipa-fxm/cob_control/pull/7.
* Considered proposals from https://github.com/ipa-fxm/cob_control/pull/7
* Added a YAML file to have the parameters as example. This folder can be deleted after integration into cob_robots package.
* Added self-collision checking. Corrected fcl bug(?): In case of simple geometric shapes the nearest_points differ from BVH models. Therefore converted simple shapes into BVH models to have the same behaviour in all cases.
* Made usage of common methods. Added defines for conversion of array access.
* Transform is done in a separate thread now. Added subscriber to CollisionObject messages to create obstacle in other nodes (e.g. Python test nodes). Added corresponding methods to process CollisionObject mesh data.
* Integrated comments of https://github.com/ipa-fxm/cob_control/pull/7. Replaced static link2collision map with URDF parser. Added class for URDF parser and create marker shapes.
* Added functions to represent a registered robot link as a mesh instead of simple shapes. Added a mapping between robot link name and mesh resource name.
* Added JLA inequality constraint to be used within the dynamic task strategy. Added checking and resetting of dynamic_reconfigure params. Corrected formatting of LSV damping.
* Separated constraints from solvers and vice versa. Added new parameters. Prettified GUI.
* Added assimp library for generic mesh file parsing. Added a parser base to specify common interfaces and methods.
* Added roslib to resolve package:// uris. Renamed typedefs. Specialized template implementation for fcl::BVHModel<fcl::RSS> > to use meshes like simple shapes. Added example code for a arm_1_collision.stl mesh.
* Made CA possible with active base. Bug fixing of solvers in case of base active. Corrected JLA constraints.
* re-arrange Parameter structs
* Added new method for dynamic tasks readjustment. Implemented prediction of distance now for vectors.
* resolve conflicts after merging ipa-fxm-mb/task_stack_prio_feature
* beautify and code-review
* Added chain recursive fk vel calculator. Corrected calculation of translational Jacobian for CA. Introduced further msg types to achieve that. Extended solvers: CA as first prio task, CA as GPM, CA as GPM with disappearing main tasks.
* Corrected dist calclation for GPM CA
* Corrected CMakeLists.txt and package.xml. Resolved dependencies.
* Added stack of tasks and further developments on GPM CA.
* Further developments.
* Implemented proposals from discussion https://github.com/ipa320/cob_control/pull/38. Removed tabs. Corrected node handles.
* Made corrections proposed in https://github.com/ipa320/cob_control/pull/38#
* - Made cob_obstacle_distance independent from testdata/robot_description.xml file.
  Only in case of the parameter /robot_description could not be read the xml file is used (e.g. for testing purposes).
  - For that added roslib as dependency.
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
* - Created a obstacle distance publisher in cob_obstacle_distance package and a subscriber in cob_twist_controller package.
  - Created registration service in cob_obstacle_distance
  - Creation of multiple CA constraints dependent on formerly registered joint regions.
* Renaming
* Contributors: Andriy Petlovanyy, ipa-fxm, ipa-fxm-mb
