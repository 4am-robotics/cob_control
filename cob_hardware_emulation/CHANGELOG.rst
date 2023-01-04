^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_hardware_emulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.21 (2023-01-04)
-------------------

0.8.20 (2022-11-17)
-------------------

0.8.19 (2022-07-29)
-------------------

0.8.18 (2022-01-12)
-------------------
* Merge pull request `#266 <https://github.com/ipa320/cob_control/issues/266>`_ from HannesBachter/fix/emulation
  fix initialpose and add laser odom in emulation
* fix some typos
* add emulation_odom_laser to CMakeLists
* generate odom laser from odom (not from twist)
* publish odom->base in emulation_odom_laser
* add emulation for laser odom
* harmonize variables (add leading underscore)
* fix initialpose handling in emulation
* Contributors: Felix Messmer, HannesBachter

0.8.17 (2021-12-23)
-------------------
* Merge pull request `#265 <https://github.com/ipa320/cob_control/issues/265>`_ from fmessmer/fix/emulation_argparse
  fixup emulation argparse
* fixup emulation argparse
* Merge pull request `#264 <https://github.com/ipa320/cob_control/issues/264>`_ from HannesBachter/feature/laser_odometry
  [no odom] make odom frame configurable
* make odom frame configurable
* Merge pull request `#263 <https://github.com/ipa320/cob_control/issues/263>`_ from fmessmer/emulation_nav
  separate emulation_nav from emulation_base
* separate emulation_nav from emulation_base
* Contributors: Felix Messmer, HannesBachter, fmessmer

0.8.16 (2021-10-19)
-------------------

0.8.15 (2021-05-17)
-------------------
* Merge pull request `#257 <https://github.com/ipa320/cob_control/issues/257>`_ from fmessmer/fix_emulated_fjt_melodic
  [melodic] fix emulated fjt
* never skip last traj point
* Contributors: Felix Messmer, fmessmer

0.8.14 (2021-05-10)
-------------------

0.8.13 (2021-04-06)
-------------------
* Merge pull request `#253 <https://github.com/ipa320/cob_control/issues/253>`_ from floweisshardt/feature/emulator_move_base_melodic
  melodic/noetic: add optional move_base interface to base emulator
* add optional move_base interface to base emulator
* Merge pull request `#249 <https://github.com/ipa320/cob_control/issues/249>`_ from fmessmer/fix/fjt_emulation_melodic
  [melodic] fix fjt emulation
* use timer for joint_states
* skip trajectory points faster than sample_rate
* introduce sample_rate_hz parameter
* Merge pull request `#247 <https://github.com/ipa320/cob_control/issues/247>`_ from fmessmer/fix_catkin_lint_melodic
  [melodic] fix catkin_lint
* fix catkin_lint
* Contributors: Felix Messmer, Florian Weisshardt, floweisshardt, fmessmer

0.8.12 (2020-10-21)
-------------------
* Merge pull request `#243 <https://github.com/ipa320/cob_control/issues/243>`_ from fmessmer/test_noetic
  test noetic
* Bump CMake version to avoid CMP0048 warning
* Merge pull request `#244 <https://github.com/ipa320/cob_control/issues/244>`_ from lindemeier/bugfix/length-bug
  [melodic] bugfix/length-bug
* sets correct length and checks for length
* Merge pull request `#240 <https://github.com/ipa320/cob_control/issues/240>`_ from benmaidel/feature/base_emu_reset_odom_melodic
  reset odometry service for base emulation
* reset odometry service for base emulation
* Contributors: Benjamin Maidel, Felix Messmer, fmessmer, tsl

0.8.11 (2020-03-21)
-------------------

0.8.10 (2020-03-18)
-------------------
* Merge pull request `#236 <https://github.com/ipa320/cob_control/issues/236>`_ from fmessmer/melodic/emulation_clock_cpp
  [melodic] add emulation_clock publisher - cpp variant
* fix boost timer + use dt_ms
* use boost timer
* configurable emulation dt
* add emulation_clock publisher - cpp variant
* Merge pull request `#234 <https://github.com/ipa320/cob_control/issues/234>`_ from fmessmer/emulation_custom_clock_melodic
  [melodic] add emulation_clock publisher
* implement emulation_clock with timer callback
* add emulation_clock publisher
* Merge pull request `#228 <https://github.com/ipa320/cob_control/issues/228>`_ from fmessmer/feature/python3_compatibility_melodic
  [ci_updates] pylint + Python3 compatibility - melodic
* fix pylint errors
* python3 compatibility via 2to3
* Merge pull request `#226 <https://github.com/ipa320/cob_control/issues/226>`_ from fmessmer/ci_updates_melodic
  [travis] ci updates - melodic
* catkin_lint fixes
* remove outdated script
* Contributors: Felix Messmer, Loy van Beek, fmessmer

0.8.1 (2019-11-07)
------------------
* Merge branch 'kinetic_dev' of github.com:ipa320/cob_control into melodic_dev
* add CHANGELOG for cob_hardware_emulation
* Merge pull request `#221 <https://github.com/ipa320/cob_control/issues/221>`_ from fmessmer/post_vacation_qa
  [WIP] post vacation qa
* fix missing dependencies
* Merge pull request `#218 <https://github.com/ipa320/cob_control/issues/218>`_ from floweisshardt/fix/emulator
  catch zero division if two trajectory points have the same time_from_start
* catch zero division if two trajectory points have the same time_from_start
* Merge pull request `#217 <https://github.com/ipa320/cob_control/issues/217>`_ from floweisshardt/emulator
  initialpose from yaml file for base emulator
* initialpose from yaml file for base emulator
* Merge pull request `#216 <https://github.com/ipa320/cob_control/issues/216>`_ from benmaidel/feature/base_emulation_initialpose
  add initialpose to emulation_base
* Merge pull request `#215 <https://github.com/ipa320/cob_control/issues/215>`_ from lindemeier/feature/1238-joint-trajectory-controller-emulator-linear-interpolation
  Feature/1238 joint trajectory controller emulator linear interpolation
* 1238 Setting joint velocity and effort to zero after eaching final trajectory point
* reset odom on initialpose
* syntax fixes
* 1238 added  service reset
* 1238 joint velocities added
* 1238 adding preempt polling
  1238 readme adjusted and small improvements
* add initialpose to emulation_base
* 1238 linear interpolation of joint states sampling the given trajectory
  1238 lerping start and goal works
  1238 Fixed error in lerp
  1238 using only local time segments for computing the interpolation weight
  1238 added more comments
* 1238 replacing timer with rospy loop rate
  + publishing joint_states with 10Hz controlled by loop rate instead of timer
* Merge pull request `#214 <https://github.com/ipa320/cob_control/issues/214>`_ from floweisshardt/feature/emulator_base
  emulator base can be used with real navigation
* migrate tf to tf2
* emulator base can be used with real navigation
* Merge pull request `#212 <https://github.com/ipa320/cob_control/issues/212>`_ from floweisshardt/emulator
  initial version of move_base emulator
* review comments
* initial version of move_base emulator
* Merge pull request `#211 <https://github.com/ipa320/cob_control/issues/211>`_ from ipa320/emulator
  add hardware_emulation package
* add hardware_emulation package
* Contributors: Benjamin Maidel, Felix Messmer, Florian Weisshardt, Thomas Lindemeier, floweisshardt, fmessmer

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

0.7.1 (2018-01-07)
------------------

0.7.0 (2017-07-18 10:50)
------------------------

0.6.15 (2017-07-18 10:30)
-------------------------

0.6.14 (2016-10-10 12:20)
-------------------------

0.6.13 (2016-10-10 11:46)
-------------------------

0.6.12 (2016-10-10 11:45)
-------------------------

0.6.11 (2016-04-01)
-------------------

0.6.10 (2015-08-31)
-------------------

0.6.9 (2015-08-25)
------------------

0.6.8 (2015-06-22)
------------------

0.6.7 (2015-06-17)
------------------

0.6.6 (2014-12-18 10:49)
------------------------

0.6.5 (2014-12-18 09:08)
------------------------

0.6.4 (2014-12-16 14:10)
------------------------

0.6.3 (2014-12-16 14:00)
------------------------

0.6.2 (2014-12-15)
------------------

0.6.1 (2014-09-22)
------------------

0.6.0 (2014-09-18)
------------------

0.5.4 (2014-08-26 10:26)
------------------------

0.1.0 (2014-08-26 10:23)
------------------------
