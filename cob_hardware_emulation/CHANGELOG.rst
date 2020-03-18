^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_hardware_emulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.10 (2020-03-18)
-------------------
* Merge pull request `#235 <https://github.com/ipa320/cob_control/issues/235>`_ from fmessmer/kinetic/emulation_clock_cpp
  [kinetic] add emulation_clock publisher - cpp variant
* fix boost timer + use dt_ms
* use boost timer
* configurable emulation dt
* add emulation_clock publisher - cpp variant
* Merge pull request `#232 <https://github.com/ipa320/cob_control/issues/232>`_ from fmessmer/emulation_custom_clock
  add emulation_clock publisher
* implement emulation_clock with timer callback
* add emulation_clock publisher
* Merge pull request `#227 <https://github.com/ipa320/cob_control/issues/227>`_ from LoyVanBeek/feature/python3_compatibility
  [ci_updates] pylint + Python3 compatibility - kinetic
* fix pylint errors
* python3 compatibility via 2to3
* Merge pull request `#225 <https://github.com/ipa320/cob_control/issues/225>`_ from fmessmer/ci_updates_kinetic
  [travis] ci updates - kinetic
* catkin_lint fixes
* Contributors: Felix Messmer, Florian Weisshardt, Loy van Beek, fmessmer

0.7.9 (2019-11-07)
------------------
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
