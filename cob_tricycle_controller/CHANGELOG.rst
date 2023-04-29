^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_tricycle_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.8.21 (2023-01-04)
-------------------

0.8.20 (2022-11-17)
-------------------

0.8.19 (2022-07-29)
-------------------

0.8.18 (2022-01-12)
-------------------

0.8.17 (2021-12-23)
-------------------

0.8.16 (2021-10-19)
-------------------

0.8.15 (2021-05-17)
-------------------

0.8.14 (2021-05-10)
-------------------

0.8.13 (2021-04-06)
-------------------

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
* Merge pull request `#192 <https://github.com/ipa320/cob_control/issues/192>`_ from fmessmer/fix/parseWheelTransform
  fix/parse wheel transform
* fix rotatory direction with proper sign
* fix rotatory direction
* fix parseWheelTransform
* debug parseWheelTransform
* Merge pull request `#191 <https://github.com/ipa320/cob_control/issues/191>`_ from fmessmer/fix_tricycle_odometry
  fix tricycle kinematic formulae
* keep steer pos for zero command
* fix tricycle kinematic formulae
* Merge pull request `#190 <https://github.com/ipa320/cob_control/issues/190>`_ from fmessmer/new_tricycle_controller_kinetic
  new tricycle controller kinetic
* fix angular.z direction
* implement additional config parameter
* fix drive velocity factor
* base radius abs value
* fix inverse kinematics, choose solution
* fix forward kinematics
* implement inverse kinematics for control_plugin
* fix forward kinematics in odom_plugin
* use PositionJointInterface for steer_joint
* fake twist_controller
* introduce cob_base_controller_utils package
* parseWheelTransform from urdf
* add fake control_plugin
* initial tricycle_controller, fake odom
* Contributors: Felix Messmer, fmessmer, ipa-fxm, robot@cob4-21
