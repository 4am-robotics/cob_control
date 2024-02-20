^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_base_controller_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.23 (2024-02-20)
-------------------

0.8.22 (2023-04-29)
-------------------
* Merge pull request `#273 <https://github.com/ipa320/cob_control/issues/273>`_ from Deleh/fix/recover_diagnostics_when_stopped
  Recover on Diagnostics When Stopped
* recover on diagnostics when stopped
* Contributors: Denis Lehmann, Felix Messmer

0.8.21 (2023-01-04)
-------------------
* Merge pull request `#272 <https://github.com/ipa320/cob_control/issues/272>`_ from Deleh/feature/diagnostic_recovery
  [WIP] Add recover from diagnostic errors
* remove yaml. yaml should be placed in X_hardware_config
* enhance recover from diagnostics
* set stuck threshold log level to debug
* also check rotation joints
* use config file for halt detector
* add recover from diagnostic errors
* Contributors: Denis Lehmann, Florian Weisshardt, floweisshardt

0.8.20 (2022-11-17)
-------------------
* Merge pull request `#271 <https://github.com/ipa320/cob_control/issues/271>`_ from Deleh/feature/halt_detector
  Add `halt_detector`
* track stuck thresholds per wheel
* add install tag for cob_halt_detector
* add warning if wheel exceeds stuck threshold
* add halt_detector
* add recovering check
* use ROS_INFO_STREAM
* simplify stop detector
* Contributors: Denis Lehmann, Felix Messmer

0.8.19 (2022-07-29)
-------------------
* Merge pull request `#269 <https://github.com/ipa320/cob_control/issues/269>`_ from fmessmer/fix/switch_recover
  start controllers after recover
* do not switch/restart odometry_controller
* fixup controller_spawn
* start controllers after recover
* Merge pull request `#268 <https://github.com/ipa320/cob_control/issues/268>`_ from fmessmer/feature/stop_detector
  feature stop detector
* only subscribe once everything is set up
* wait for required services
* add missing launch files
* add stop_detector
* Contributors: Felix Messmer, fmessmer

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

0.7.6 (2019-06-07)
------------------

0.7.5 (2019-05-20)
------------------

0.7.4 (2019-04-05)
------------------
* Merge pull request `#200 <https://github.com/ipa320/cob_control/issues/200>`_ from iirob/stuck_detector_shutdown
  Added shutdown in stuck detector
* Added shutdown in stuck detector
* Contributors: Felix Messmer, andreeatulbure

0.7.3 (2019-03-14)
------------------
* Merge pull request `#199 <https://github.com/ipa320/cob_control/issues/199>`_ from fmessmer/spin_detector
  Spin detector
* Revert "tmp: add spin_detector to launch"
  This reverts commit d08388aad4e0910f933f8439faff2ce97ba3c1a4.
* tmp: add spin_detector to launch
* allow shutdown vs. halt
* add spin_detector
* Merge pull request `#196 <https://github.com/ipa320/cob_control/issues/196>`_ from fmessmer/split_parseWheelTransform
  controller specific parseWheelTransform
* split parseWheelTransform
* Merge pull request `#192 <https://github.com/ipa320/cob_control/issues/192>`_ from fmessmer/fix/parseWheelTransform
  fix/parse wheel transform
* fix rotatory direction with proper sign
* fix parseWheelTransform
* debug parseWheelTransform
* Merge pull request `#190 <https://github.com/ipa320/cob_control/issues/190>`_ from fmessmer/new_tricycle_controller_kinetic
  new tricycle controller kinetic
* fix launch file
* implement inverse kinematics for control_plugin
* introduce cob_base_controller_utils package
* Contributors: Felix Messmer, fmessmer
