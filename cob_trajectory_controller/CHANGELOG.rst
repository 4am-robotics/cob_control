^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_trajectory_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.6.8 (2015-06-17)
------------------

0.6.7 (2015-06-17)
------------------
* fix topic names
* replace brics_actuator
* use new Trigger from std_srvs
* cleanup/replace cob_srvs
* fix getParam, fix ActionServer auto_start
* adapt cob_trajecory_controller to new namespaces
* remove obsolete files
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
* fixed preemtion state in run method - trajectory eecution should be stopped if the action is in preemption state
* Contributors: Benjamin Maidel

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
* fixed CMakeLists
* cherry-pick
* removed a lot of code related to packages not available in hydro anymore
* try to use cob_trajectory_controller with gazebo simulation
* output long unsigned variables correctly
* changes for hydro
* cleaned up CMakeLists and added install directives
* added missing message_gen deps
* further modifications for catkin, now everything is compiling and linking
* futher include and linkpath modifications
* compiling but still some linker errors
* Second catkinization push
* First catkinization, still need to update some CMakeLists.txt
* Fixed dependencies
* Groovy migartion
* test
* support for runtime modification of trajectories, working but not entirely smooth yet
* initial version of support for runtime modification of trajectories, still needs some improvements
* removed unused topic
* switched from pr2_controllers_msgs::JointTrajectoryAction to control_msgs::FollowJointTrajectory
* add return value
* remove hardcoded arm
* fixed action result for stopping trajectory
* abort old trajectory when new one is issued
* trajectory controller fixes tested on robot
* Rejecting with timeout if arm doesn't go into velocity mode, aborting action on control failure, see http://www.care-o-bot-research.org/trac/ticket/275
* add some TODO comments
* remove blub
* changes for fuerte compatibility
* some small changes for frequency refactoring
* changed controller to take frequency out of configuration
* update manifest desription
* renamed services of trajectory_controller
* remove deprecated tests
* removed deprecated yaml and launch files
* fixed simulation bug
* added services to set velocities and accelerations in the cob_trajectory_controller
* removed compiler warnings
* smore interfaces for testing in simulation
* modifications for trajectory_controller to work with tray and torso
* merge
* change speed
* Merge branch 'master' of github.com:ipa-fmw/cob_driver
* more beautiful configuration settings
* configuration of cob_trajectory controller
* remove compiler warnings
* additional follow trajectory action in cob_trajectory_controller
* added roslaunch tests
* after debugging script
* modifications for fetch and carry
* fetch and carry on cob3-3
* added different safety issues, changed interface to be stateful
* stopping state running
* fixed action state stuff
* modified action server to be preemted
* added stop service to controller
* fix merge
* merge
* merge
* rearranging cob_camera_sensors launch files
* update for icob and schunk arm
* changed structure of cob_trajectory_controller to standard cob structure
* the new powerchain implementation with slight modifications regarding timing, trajectory controller tested on Schunk LWA3 hardware
* working version of spline trajectory controller, tested in simulation with dashboard, testing with real hardware and path planning trajectories still has to be done
* moved trajectory controller to driver stack
* Contributors: Alexander Bubeck, Felix Messmer, Frederik Hegger, Richard Bormann, abubeck, cob3-5, cpc-pk, ipa-fmw, ipa-fmw-ws, ipa-fxm, ipa-nhg, ipa-taj
