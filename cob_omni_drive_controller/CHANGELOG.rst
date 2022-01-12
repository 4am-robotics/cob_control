^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_omni_drive_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.18 (2022-01-12)
-------------------

0.8.17 (2021-12-23)
-------------------

0.8.16 (2021-10-19)
-------------------
* Merge pull request `#259 <https://github.com/ipa320/cob_control/issues/259>`_ from HannesBachter/feature/invert_odom
  [melodic] Feature/invert odom
* rename to invert_odom_tf
* switch frame_id and child_frame_id when invert_odom
* set invert_odom param to false by default
* introduce invert_odom param
* publish inverted transform for odometry
* Contributors: Felix Messmer, HannesBachter

0.8.15 (2021-05-17)
-------------------

0.8.14 (2021-05-10)
-------------------

0.8.13 (2021-04-06)
-------------------
* Merge pull request `#251 <https://github.com/ipa320/cob_control/issues/251>`_ from fmessmer/melodic/TF_REPEATED_DATA
  [melodic] add check and logs for TF_REPEATED_DATA
* add check and logs for TF_REPEATED_DATA
* Merge pull request `#247 <https://github.com/ipa320/cob_control/issues/247>`_ from fmessmer/fix_catkin_lint_melodic
  [melodic] fix catkin_lint
* fix catkin_lint
* Contributors: Felix Messmer, fmessmer

0.8.12 (2020-10-21)
-------------------
* Merge pull request `#243 <https://github.com/ipa320/cob_control/issues/243>`_ from fmessmer/test_noetic
  test noetic
* Bump CMake version to avoid CMP0048 warning
* Contributors: Felix Messmer, fmessmer

0.8.11 (2020-03-21)
-------------------

0.8.10 (2020-03-18)
-------------------
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
* Merge pull request `#208 <https://github.com/ipa320/cob_control/issues/208>`_ from ipa-jba/melodic_dev
  [Melodic] melodify
* use urdf::JointConstSharedPtr instead of boost...
* Contributors: Felix Messmer, Jannik Abbenseth

0.7.6 (2019-06-07)
------------------

0.7.5 (2019-05-20)
------------------

0.7.4 (2019-04-05)
------------------

0.7.3 (2019-03-14)
------------------
* Merge pull request `#196 <https://github.com/ipa320/cob_control/issues/196>`_ from fmessmer/split_parseWheelTransform
  controller specific parseWheelTransform
* split parseWheelTransform
* Merge pull request `#190 <https://github.com/ipa320/cob_control/issues/190>`_ from fmessmer/new_tricycle_controller_kinetic
  new tricycle controller kinetic
* implement additional config parameter
* introduce cob_base_controller_utils package
* parseWheelTransform from urdf
* missing includes for cob_omni_drive_controller
* Contributors: Felix Messmer, fmessmer, ipa-fxm

0.7.2 (2018-07-21)
------------------
* update maintainer
* Merge pull request `#177 <https://github.com/ipa320/cob_control/issues/177>`_ from benmaidel/fix/wheel_transform
  fix wheel transformation from urdf
* user robotmodel root link for wheel transformation
* fixed tf2 includes
* read whole transformation chain between wheel and base_link
* Merge pull request `#186 <https://github.com/ipa320/cob_control/issues/186>`_ from ipa-fxm/kinetic/fix/max_wheel_rate_from_urdf
  read max_wheel_rates from urdf
* read max_wheel_rates from urdf
* Merge pull request `#140 <https://github.com/ipa320/cob_control/issues/140>`_ from ipa-mdl/feature/wheel-multi-controller-refactored
  [kinetic] wheel multi controller refactored
* add multi interface wheel controller
* Contributors: Benjamin Maidel, Felix Messmer, Mathias Lüdtke, fmessmer, ipa-fxm

0.7.1 (2018-01-07)
------------------
* Merge remote-tracking branch 'origin/kinetic_release_candidate' into kinetic_dev
* Merge pull request `#169 <https://github.com/ipa320/cob_control/issues/169>`_ from ipa-fxm/kinetic_updates_indigo
  Kinetic updates indigo
* Merge branch 'indigo_dev' of github.com:ipa320/cob_control into kinetic_dev
  Conflicts:
  .travis.yml
* Merge pull request `#164 <https://github.com/ipa320/cob_control/issues/164>`_ from ipa-fxm/update_maintainer
  update maintainer
* update maintainer
* Merge pull request `#159 <https://github.com/ipa320/cob_control/issues/159>`_ from ipa-fxm/APACHE_license
  use license apache 2.0
* use license apache 2.0
* Contributors: Felix Messmer, ipa-fxm, ipa-uhr-mk

0.7.0 (2017-07-18)
------------------

0.6.15 (2017-07-18)
-------------------
* remove sympy implementation and dependency
* implement setting pos ctrl params with  dynamic_reconfigure
* dynamic reconfigure config for SteerCtrl parameter
* refactored cob_omni_wheel_controller classes
* add test scripts
* manually fix changelog
* Contributors: Felix Messmer, Mathias Lüdtke, ipa-fxm

0.6.14 (2016-10-10)
-------------------

0.6.13 (2016-10-10)
-------------------

0.6.12 (2016-10-10)
-------------------
* added stuck detector node
* publish WheelCommands, rate devided by pub_divider
* introduced WheelCommand, outputs state and steer target error
* Contributors: Mathias Lüdtke

0.6.11 (2016-04-01)
-------------------
* remove leading slashes and use odom as default
* configurable odometry_controller
* [hotfix] compile error
* Contributors: ipa-fmw, ipa-fxm

0.6.10 (2015-08-31)
-------------------

0.6.9 (2015-08-25)
------------------
* boost revision
* explicit dependency to boost
* more dependency fixes according to review comments
* explicit dependency to boost
* remove trailing whitespaces
* migrate to package format 2
* sort dependencies
* review dependencies
* Contributors: ipa-fxm

0.6.8 (2015-06-17)
------------------

0.6.7 (2015-06-17)
------------------
* migrated to std_srvs/Trigger
* improved realtime behavoíour, no updates can get lost anymore
* added dependency on cob_srvs, fixed c&p bug
* do not reset odometry on restarts
* fixed reset service: compiles again and thread/RT-safe
* added service to reset odometry to zero
* rename topic as agreed
* update examples
* support for wheel struct
* Handle XmlRpcValue parsing error, special case for double
* fix for logic bug
* init to neutral position works now
* so not reset target on reset(), should be done externally if needed
* fixed copy&paste bugs
* added controller type in examples
* try_read is now read_with_default
* validity checks  for wheel_radius
* introduced read_optional, fixed URDF parsing
* steer_name and drive_name are now read from steer and drive paramater (as in examples)
* logic bug
* publish_rate was used as duration, added validity check
* advertise odom topic
* UndercarriageCtrl::reset got lost during split-up
* refactored WheelController to improve locking behaviour, implement timeout and limit checks
* reset Target to zero on reset()
* expose UndercarriageCtrl::limitValue with limit validity check
* updated examples
* splitted UndercarriageCtrlGeom into UndercarriageGeomBase, UndercarriageGeom and UndercarriageCtrl
* enforced lower camel case methods
* removed parseIniFiles
* added example yamls
* implemented parseWheelParams with URDF look-up
* limit steer and drive rate if specified
* added WheelParams::dSteerDriveCoupling, WheelData::dFactorVel is now filled automatically
* removed debugging output
* adaptet constructor of WheelData to set the neutralPos of a wheel
* Merge branch 'omni_wheel' of https://github.com/olgen2013/cob_control into omni_wheel
* fix assignment bug
* online/robot modifications
* fixed assignment bug and added console out put for online-testing
* improved INI parsing
* migrated to shared implementation of GeomController
* verbose exception handling
* library path was wrong
* Contributors: Florian Weisshardt, Joshua Hampp, Mathias Lüdtke, ipa-fxm, mig-jg

0.6.6 (2014-12-18)
------------------

0.6.5 (2014-12-18)
------------------
* Merge branch 'indigo_dev' into indigo_release_candidate
* add dep
* Contributors: Florian Weisshardt

0.6.4 (2014-12-16)
------------------
* Merge branch 'indigo_dev' into indigo_release_candidate
* update deps
* Contributors: Florian Weisshardt

0.6.3 (2014-12-16)
------------------

0.6.2 (2014-12-15)
------------------
* added plugin desctiption and install tags
* added plugins
* added GeomController Helper
* further dependencies
* added Boost dependency
* added OdometryTracker
* removed unused member
* added SI function to PlatformState
* added INI file parsing
* added reset to UndercarriageCtrlGeom/::Wheel
* restructured and optimised version
* simplified GetNewCtrlStateSteerDriveSetValues
* got rid of m_dCmdRotVelRadS
* refactored GetActualPltfVelocityVelocity
* downstripped version
* introduced resetController
* version without IniFile and MathSup
* original version of UndercarriageCtrlGeom
* Contributors: Mathias Lüdtke

* added plugin desctiption and install tags
* added plugins
* added GeomController Helper
* further dependencies
* added Boost dependency
* added OdometryTracker
* removed unused member
* added SI function to PlatformState
* added INI file parsing
* added reset to UndercarriageCtrlGeom/::Wheel
* restructured and optimised version
* simplified GetNewCtrlStateSteerDriveSetValues
* got rid of m_dCmdRotVelRadS
* refactored GetActualPltfVelocityVelocity
* downstripped version
* introduced resetController
* version without IniFile and MathSup
* original version of UndercarriageCtrlGeom
* Contributors: Mathias Lüdtke

0.6.1 (2014-09-22)
------------------

0.6.0 (2014-09-18)
------------------

0.5.4 (2014-08-26 10:26)
------------------------

0.1.0 (2014-08-26 10:23)
------------------------
