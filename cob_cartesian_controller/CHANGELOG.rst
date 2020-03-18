^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_cartesian_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.10 (2020-03-18)
-------------------
* Merge pull request `#228 <https://github.com/ipa320/cob_control/issues/228>`_ from fmessmer/feature/python3_compatibility_melodic
  [ci_updates] pylint + Python3 compatibility - melodic
* disable simple_script_server import error
* fix modules
* fix pylint errors
* Merge pull request `#226 <https://github.com/ipa320/cob_control/issues/226>`_ from fmessmer/ci_updates_melodic
  [travis] ci updates - melodic
* catkin_lint fixes
* Contributors: Felix Messmer, fmessmer

0.8.1 (2019-11-07)
------------------

0.8.0 (2019-08-09)
------------------

0.7.8 (2019-08-09)
------------------

0.7.7 (2019-08-06)
------------------

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

0.6.15 (2017-07-18)
-------------------
* cleanup leftovers
* use xacro --inorder
* remove inline keywords
* manually fix changelog
* optimize output
* clear preview
* remove obsolete script
* add install tags
* fix and harmonize scripts
* broadcast target_frame before tracking
* Contributors: ipa-fxm

0.6.14 (2016-10-10)
-------------------

0.6.13 (2016-10-10)
-------------------

0.6.12 (2016-10-10)
-------------------

0.6.11 (2016-04-01)
-------------------
* remove lwa4d test scripts
* use ros::Time::now
* use const in function parameters
* remove movePTP
* minor styling
* remove unused Pose parameter
* significantly simplify function parameters for getTrajectory and getProfileTimings
* re-order vel-acc
* get rid of ProfileTimings.ok
* move identical functions to base class
* harmonizing
* remove obsolete calcTe\_
* minor styling
* roslint cob_cartesian_controller
* towards code styling consistency
* adjust descriptions ins license plate
* adjust description
* Final version
* Almost finalized version
* Implemented move_circ
* remove obsolete files
* adjust service type in cartesian controller
* LWA4D test scripts
* New quaternion interpolation
* Linear interpolation works fine now. There's still a bug in quaternion interpolation.
* Bug fixes + code reduction
* apply change in parameter name
* code reduction part 2.
* forgot the cpp files
* added new headers
* Code reduction part 1.. there's still a bug in ramp profiles.
* Fixed a bug in sort algorithm and profile interpolation
* new example script
* restructured, introducing base class for profile generator
* renaming: hardware_interface to controller_interface
* update trajectory_hardware_interface
* handle base_compensation in kinematic_extension enum
* implement JointTrajectory hardware_interface for twist_controller
* Contributors: Marco Bezzon, ipa-fxm, ipa-fxm-cm

0.6.10 (2015-08-31)
-------------------

0.6.9 (2015-08-25)
------------------
* Corrections integrated from PR: [WIP] Finalizing/Testing of TwistController features (`#51 <https://github.com/ipa-fxm/cob_control/issues/51>`_). Renaming from frame_to_collision to link_to_collision.
* boost revision
* more dependency fixes according to review comments
* Resolved merge conflicts.
* more fixes for migration afer merge
* merge with package_xml_2
* remove trailing whitespaces
* migrate to package format 2
* - Removed unnecessary function.
  - Prepared configuration params.
* resolve possible endless-loop
* consider PR review comments
* review dependencies
* updates from ipa-fxm-mb
* updates from ipa320
* Implemented Python package to set dyn_reconfigure params. Made test_move_around_torus use of this class.
* missing add_dependencies
* sort dependencies
* review dependencies
* print result, use sci in test scripts
* Created test, Removed commends, Removed output.
* Fixed bugs in cartesian_controller: waitFor last available transform else extrapolation error; send always a new constructed StampedTransform instead of using an already existent one, else end-effector is decoupled from manipulator and other confusing things happen...; Added responsible node to tf error msg.
* fix cartesian_interface
* first draft for python interface
* re-work message structure use pose and frame_id, proper handling transformation to root_frame
* added publisher for path preview
* split and restructure ProfileGenerator
* simplify data_type conversion, cleanup
* re-work of ActionServer: more failure handling
* draft for example
* replace .prog files with according .py scripts, use rospy.sleep() instead of holdPosition action
* get rid of holdPosition, replaced by rospy.sleep()
* restructure and simplify cartesian_controller_utils, beautification
* Further tests and adaptations for test.
* Made cob_cartesian_controller work again: Added CartesianController::convertMoveLinRelToAbs method again (why removed?)
* Added generated const from .cfg; Styling
* Merge with code style fixes.
* code styling cob_cartesian_controller
* renamed variable
* restructured cartesian controller with action interface
* added action server
* fix install tags
* restructured functions
* added headers..
* restructured
* Contributors: ipa-fxm, ipa-fxm-cm, ipa-fxm-mb

0.6.8 (2015-06-17)
------------------
* merge with release candidate
* package renaming: cob_path_broadcaster -> cob_cartesian_controller
* Contributors: ipa-fxm

0.6.7 (2015-06-17)
------------------
* restructure namespaces for parameters of cartesian controllers
* beautify CMakeLists
* remove obsolete files
* testing
* missing files
* new prog files
* new features
* new files
* new movement files
* clean up cob_path_broadcaster
* new features
* cleaned up
* update merged2
* feature reachable_goal
* fixed a bug in circular interpolation
* test
* Modified for the new structure
* Contributors: Christian Ehrmann, ipa-fxm, ipa-fxm-cm, ipa-fxm-fm

0.6.6 (2014-12-18)
------------------

0.6.5 (2014-12-18)
------------------

0.6.4 (2014-12-16)
------------------

0.6.3 (2014-12-16)
------------------

0.6.2 (2014-12-15)
------------------
* Merge branch 'indigo_dev' into indigo_release_candidate
* few more changes after testing new structure
* cleaning up
* merge_cm
* temporary commit
* Cleaned up and fixed some functions
* Cleaned up and fixed some functions
* Contributors: Florian Weisshardt, ipa-fxm, ipa-fxm-cm

0.6.1 (2014-09-22)
------------------

0.6.0 (2014-09-18)
------------------
* erge branch 'velocity_interface_controller_indigo' of github.com:ipa-fxm-cm/cob_control into velocity_interface_controller_indigo
* new command move_circ added
* New CMake File and cob_articulation got new functions
* fixed install tags
* catkin_lint'ing
* merge with velocity_interface_controller (hydro)
* fix ascii characters
* Added xml parser for motion primitives move_ptp and move_lin
* beautify package xml and CMakeLists
* new package cob_path_broadcaster
* new package cob_path_broadcaster
* Contributors: Christoph Mark, Felix Messmer, ipa-fxm, ipa-fxm-cm

0.5.4 (2014-08-26 10:26)
------------------------

0.1.0 (2014-08-26 10:23)
------------------------
