^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bwi_scavenger
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.12 (2016-09-14)
-------------------

0.3.11 (2016-08-27)
-------------------

0.3.10 (2016-08-15)
-------------------

0.3.9 (2016-08-05)
------------------
* bwi_scavenger: add missing visualization_msgs dependency (`#61 <https://github.com/utexas-bwi/bwi_common/issues/61>`_)
* commented out image uploading in human following
* final change before demo, hopefully
* changed image topic name from image_color to image_raw
* uniformed naming strategy of certificate images
* fixed white-board not stopping problem
* fixed bugs; todo: 'stop' button sometimes doesn't stop moving in whiteboard task
* fixed human following 'no passive moving' problem
* enabled robot to say 'following' while following a person
* fixed fetch-object problem: robot doesn't stop after user presses the button
* Merge branch 'shiqi/scavenger' of https://github.com/utexas-bwi/bwi_common into shiqi/scavenger
* human following ends when task completed
* fixed color shirt detection, shirt height problem
* removed directory, leaving only file name of certificate
* added a path to certificate messages from scav to virtour
* added color into color shirt printout on virtour
* fixed bugs, and demo worked well
* added scav_srv into scavenger.cpp
* added a new scavenger hunt service that can pause/resume current task
* Merge remote-tracking branch 'origin/master' into shiqi/scavenger
* scavenger: removed the blocking thread.join() line
* enabled scavenger to keep publishing task status to bwi_virtour
* fixed bug of scavenger status not publishing
* bwi_msgs: added a ScavStatus msg
* activated all four tasks
* Added scp log files to remote (hypnotoad) location
* Took out spin. It's too easy to be stuck in a loop with false detections
* Finished implemented more fluid human following given the person's last seen location
* Changed human following motion to actionlib version. 
* scavenger added image saver to human following
* Added stop in front of person code for human following
* added traveled distance into rviz
* enabled robot to draw two trajectories: robot and human
* added a tool for drawing robot trajectories
* ScavTaskHumanFollowing: simply approaching where human was first detected
* added a new task ScavTaskHumanFollowing; to be tested on robots
* configuration used for scavenger hunt demo
* fixed a problem about parameter passing
* Contributors: Jack O'Quin, Shih-Yun Lo, Shiqi Zhang, William Xie

0.3.8 (2016-06-06)
------------------
* use GlobalPlanner instead of navfn
* door map for corpp experiments
* Contributors: Piyush Khandelwal, Shiqi Zhang

0.3.7 (2015-09-22)
------------------

0.3.6 (2015-08-25)
------------------
* bwi_scavenger: remove segbot_gui and bwi_rlg references (`#30 <https://github.com/utexas-bwi/bwi_common/issues/30>`_)
* bwi_scavenger: work-around for yaml-cpp problem on Saucy (`#35 <https://github.com/utexas-bwi/bwi_common/issues/35>`_)
* Contributors: Jack O'Quin

0.3.5 (2015-08-22)
------------------
* bwi_scavenger: fix catkin lint and build problems (`#31 <https://github.com/utexas-bwi/bwi_common/issues/31>`_)
* Contributors: Jack O'Quin

0.3.4 (2015-08-19)
------------------
* restore missing package versions
* added maintainers, and minor changes in package.xml files
* fixed yaml-cpp version problem
* Contributors: Jack O'Quin, Shiqi Zhang

0.3.3 (2015-08-05)
------------------
* added dependencies of move_base_msgs
* fixed `#26 <https://github.com/utexas-bwi/bwi_common/issues/26>`_: system dependency problem about yaml-cpp
* moved scavenger hunt package from bwi_experimental to bwi_common
* Contributors: Shiqi Zhang
