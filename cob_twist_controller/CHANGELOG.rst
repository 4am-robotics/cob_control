^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_twist_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.6 (2014-12-18)
------------------
* remove dep to cob_srvs and std_srvs
* Contributors: Florian Weisshardt

0.6.5 (2014-12-18)
------------------
* Merge branch 'indigo_dev' into indigo_release_candidate
* add dep
* Contributors: Florian Weisshardt

0.6.4 (2014-12-16)
------------------

0.6.3 (2014-12-16)
------------------
* add dependency to nav_msgs
* Contributors: Florian Weisshardt

0.6.2 (2014-12-15)
------------------
* Merge branch 'indigo_dev' into indigo_release_candidate
* fix twist_control dimensions for any-DoF
* merge with fxm-cm
* merge with fxm-fm
* cleaning up
* branch with features for merging
* topics according to new structure
* remove brics_actuator
* more topic renaming according to new structure
* renaming debug topic
* adapt namespaces for cartesian_controller to new structure
* dynamic reconfigure
* revision of cob_twist_controller
* merge_cm
* merge_fm
* temporary commit
* temporary commit
* changes in initialization
* restructure test_twist publisher scripts
* fix twist_controller to be usable without base again
* able to add base DoFs to Jacobian solver - first tests - needs more debugging
* null-space syncMM
* add test script for twist_stamped
* able to apply twists wrt to various coordinate system orientations
* cleanup, restructure and fix
* missing include
* merge with fxm-fm + clean up
* add twist publisher script
* add output publisher
* cleaning up
* beautify
* Add fixes provided by @ipa-fxm-fm
* fix controller and add damping
* add twist publisher script
* add output publisher
* Add fixes provided by @ipa-fxm-fm
* Contributors: Florian Weisshardt, ipa-fxm, ipa-fxm-cm, ipa-fxm-fm

0.6.1 (2014-09-22)
------------------

0.5.4 (2014-08-26)
------------------
* fix dependency-hell on multiple cores
* moved cob_twist_controller
* Contributors: Alexander Bubeck, ipa-fxm
