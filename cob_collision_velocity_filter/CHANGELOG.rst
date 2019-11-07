^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_collision_velocity_filter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.9 (2019-11-07)
------------------

0.7.8 (2019-08-09)
------------------

0.7.7 (2019-08-06)
------------------
* Merge pull request `#208 <https://github.com/ipa320/cob_control/issues/208>`_ from ipa-jba/melodic_dev
  [Melodic] melodify
* initialize tf buffer properly
* move include to where it is needed
* add missing tf2_ros dependency
* tf2ify/melodify collision_velocity_filter
* Contributors: Felix Messmer, Jannik Abbenseth

0.7.6 (2019-06-07)
------------------
* Merge pull request `#207 <https://github.com/ipa320/cob_control/issues/207>`_ from fmessmer/fix_collision_velocity_filter
  fix collision_velocity_filter parameter initialization
* fix collision_velocity_filter parameter initializatin
* Contributors: Felix Messmer, fmessmer

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
* manually fix changelog
* Contributors: ipa-fxm

0.6.14 (2016-10-10)
-------------------

0.6.13 (2016-10-10)
-------------------

0.6.12 (2016-10-10)
-------------------  
* Integrate costmap in velocity filter
* Added public nodehandle
* Renamed incoming command topic to command_in
* Code reformat with ROS Style
* Little fixes
* Removed parameter for topic
* Cleanup
* Integrated costmap in collision_velocity_filter node
* Intermediate state
* Added parameter "teleop_topic" and removed costmap running check for now.
* Added costmap_2d to CMakeLists
* Integrated costmap in cob_collision_velocity_filter node
* Integrate costmap in cob_collision_velocity_filter (first commit)
* Contributors: mig-em

0.6.11 (2016-04-01)
-------------------

0.6.10 (2015-08-31)
-------------------

0.6.9 (2015-08-25)
------------------
* boost revision
* do not install headers in executable-only packages
* explicit dependency to boost
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
* beautify CMakeLists
* Contributors: ipa-fxm

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

0.6.1 (2014-09-22)
------------------

0.5.3 (2014-03-31)
------------------
* install tags
* Contributors: ipa-fxm

0.5.2 (2014-03-20)
------------------

0.5.1 (2014-03-20)
------------------
* add definitions to get rid of compiler warning
* Corrected compile error. See pull-request `#80 <https://github.com/ipa320/cob_driver/issues/80>`_ in original repo.
* merge changes from frederikhegger, `#80 <https://github.com/ipa320/cob_driver/issues/80>`_
* change from gencpp (for msgs and srvs) to gencfg (for dyn recfg)
* changes for hydro
* Installation stuff
* cleaned up CMakeLists and added install directives
* further modifications for catkin, now everything is compiling and linking
* compiling but still some linker errors
* Second catkinization push
* First catkinization, still need to update some CMakeLists.txt
* Color interpolation added to the velocity limited marker
* set max vel to 0.2
* set max velocity for marker to 0.3
* A new marker shown when rotation speed is limited
* remove debug output
* add markers to veclocity filter
* reduce max range
* fix divide by zero bug
* make cob_collision_velocity_filter dynamically reconfigureable
* move src files
* move launch and config files to cob_robots
* add laser_scan_top_clearing as additional observation_source
* add scan_top_clearing topic
* add ramp to limit acceleration
* change documentation
* merge launch files and move configs to same folder
* put GetFootprint service into footprint observer
  instead of SetFootprint service in collision velocity filter
  fix namespace problems
* only use laser data for velocity filter
* integrate safe velocity controller by default
* add missing dependency
* moved safe base movement to driver stack
* Contributors: Alexander Bubeck, Florian Weißhardt, Frederik Hegger, IPR-SR2, abubeck, but-spanel, ipa-fmw, ipa-mig, mig, srs
