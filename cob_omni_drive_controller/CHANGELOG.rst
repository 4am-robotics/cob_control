^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_omni_drive_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
