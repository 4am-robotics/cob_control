^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_frame_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.10 (2020-03-18)
-------------------
* Merge pull request `#228 <https://github.com/ipa320/cob_control/issues/228>`_ from fmessmer/feature/python3_compatibility_melodic
  [ci_updates] pylint + Python3 compatibility - melodic
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

0.7.7 (2019-08-06)
------------------

0.7.6 (2019-06-07)
------------------

0.7.5 (2019-05-20)
------------------
* Merge pull request `#206 <https://github.com/ipa320/cob_control/issues/206>`_ from fmessmer/improve_lookat_extension
  improve lookat extension
* do not start lookat if KinematicExtension service failed
* Contributors: Felix Messmer, fmessmer

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
* Merge pull request `#168 <https://github.com/ipa320/cob_control/issues/168>`_ from ipa-aep/bugfix/checkInfinitesimalTwist
  Bugfix/check infinitesimal twist
* fix wrong axes bug in Infinitesimal distance check
* add rotational distance computation to abortion criteria
* Merge pull request `#159 <https://github.com/ipa320/cob_control/issues/159>`_ from ipa-fxm/APACHE_license
  use license apache 2.0
* Merge pull request `#160 <https://github.com/ipa320/cob_control/issues/160>`_ from ipa-aep/feature/frametracker_pid_debugging
  add publisher for controller error
* remove terminal output of period
* add publisher for controller error
* use license apache 2.0
* Contributors: Alexander Pekarovskiy, Felix Messmer, ipa-fxm, ipa-uhr-mk

0.7.0 (2017-07-18)
------------------

0.6.15 (2017-07-18)
-------------------
* Merge pull request `#122 <https://github.com/ipa320/cob_control/issues/122>`_ from ipa-fxm/indigo_roslint
  roslint some packages
* starting to roslint some packages
* manually fix changelog
* Contributors: Nadia Hammoudeh García, ipa-fxm

0.6.14 (2016-10-10)
-------------------

0.6.13 (2016-10-10)
-------------------

0.6.12 (2016-10-10)
-------------------

0.6.11 (2016-04-01)
-------------------
* add frameExists check to FrameTracker
* updateMarker on startLookat
* cleanup roslint
* introduce cfg-parameter enable_abortion_checking
* Added waitForTransform in the getTransform function.. It threw an transform exception
* Merge branch 'refactor_profile_generator' of github.com:ipa-fxm-cm/cob_control into test_new_cartesian_controller
  Conflicts:
  cob_frame_tracker/src/cob_frame_tracker.cpp
* parameterizable scaling_factor
* Linear interpolation works fine now. There's still a bug in quaternion interpolation.
* only reset lookat extension
* prepare interactive_frame_target for being used with lookat
* prepare frame_tracker for being used with lookat
* constant publish rate
* working on log output
* parameterizable marker_scale
* re-activate publishHoldTwist, fix typo
* publish ZeroTwist, root/tip frame selection
* introduce scaling_factor and dead_man
* simple spacenav commander
* temporarily undo publishHoldTwist
* Contributors: Marco Bezzon, ipa-fxm, ipa-fxm-cm

0.6.10 (2015-08-31)
-------------------

0.6.9 (2015-08-25)
------------------
* Corrections integrated from PR: [WIP] Finalizing/Testing of TwistController features (`#51 <https://github.com/ipa-fxm/cob_control/issues/51>`_). Renaming from frame_to_collision to link_to_collision.
* - Made some changes for test. - Decreased Duration time for markers. - Corrected pose update for self collision check frames. - Added new scripts.
* boost revision
* Some preparations for test: IMarker smaller, Alpha settings, More scripts. Default value for Frame Tracker params.
* - Added functionality to hold twist in case of deviation of cart. distance gets to large.
* add actionlib
* explicit dependency to boost
* more fixes for migration afer merge
* remove trailing whitespaces
* migrate to package format 2
* review dependencies
* updates from ipa320
* cleanup
* sort dependencies
* review dependencies
* Fixed bugs in cartesian_controller: waitFor last available transform else extrapolation error; send always a new constructed StampedTransform instead of using an already existent one, else end-effector is decoupled from manipulator and other confusing things happen...; Added responsible node to tf error msg.
* code styling cob_frame_tracker
* Implemented proposals from discussion https://github.com/ipa320/cob_control/pull/38. Removed tabs. Corrected node handles.
* Removed tracking error publisher / subscriber and removed additional p gain for PD-Control (already done in FrameTracker with PID controller)
* bug fix
* - Added a publisher for the tracking errors to send them to cob_twist_controller
  - Added a subscriber to collect the errors and put them to the solver.
  - Added a parameter to set the p gain. If 0.0 old behavior is active (default value).
* Contributors: ipa-fxm, ipa-fxm-cm, ipa-fxm-mb

0.6.8 (2015-06-17)
------------------

0.6.7 (2015-06-17)
------------------
* restructure namespaces for parameters of cartesian controllers
* complete revision of frame_tracker structure and action server
* cleanup/replace cob_srvs
* beautify CMakeLists
* cleanup dependencies
* use individual pid parameters to reduce output
* remove obsolet files
* proper expert interaction mode
* delete obsolete files
* merged running
* frame_tracker after merge
* merge with fxm - not working
* remove obsolete files
* last update
* attach menu to marker, beautify
* MOVE_ROTATE_3D for interactive markers
* update before creating new branch
* gitignore
* gitignore
* update working frame_tracker
* correct cmake
* update after merge
* merge with fm-cm-ce
* cleaning up
* more efficient c++ version of interactive_frame_target
* diff twist calc
* update frame_tracker
* frame_tracker_new
* Merge branch 'indigo_dev' of https://github.com/ipa320/cob_control into fm_cm_merged_new
* new rqt_features
* test
* cleaned up again
* Cleaned up
* Contributors: Christian Ehrmann, ipa-fxm, ipa-fxm-cm

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
* adapt namespaces for cartesian_controller to new structure
* merge_cm
* temporary commit
* changes in initialization
* temporarily revert to non-feedforward pid_controllers
* null-space syncMM
* Add PID for each translatorial Axes
* Add PID for each translatorial Axes
* Contributors: ipa-fxm, ipa-fxm-cm

0.6.1 (2014-09-22)
------------------

0.6.0 (2014-09-18)
------------------
* update version number
* update changelog
* beautify package xml and CMakeLists
* add missing dependencies
* update interactive marker when not tracking
* introducing PID for frame_tracker, generalization of interactive_frame_target
* new menu entry: reset_tracking
* make frame_tracker and interactive_marker more generic to be used with non-lookat twist_control
* moved frame_tracker to separate package
* update interactive marker when not tracking
* introducing PID for frame_tracker, generalization of interactive_frame_target
* new menu entry: reset_tracking
* make frame_tracker and interactive_marker more generic to be used with non-lookat twist_control
* moved frame_tracker to separate package
* Contributors: Felix Messmer, Florian Weisshardt, ipa-fxm

0.5.4 (2014-08-26 10:26)
------------------------

0.1.0 (2014-08-26 10:23)
------------------------
