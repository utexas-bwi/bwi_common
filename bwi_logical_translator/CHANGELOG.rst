^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bwi_logical_translator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

forthcoming
-----------
* Moved package to bwi_common, changing name to ``bwi_logical_translator`` 
  (`#33 <https://github.com/utexas-bwi/bwi_common/issues/33>`_).


0.3.5 (2016-08-27)
------------------

0.3.4 (2016-08-08)
------------------
* Updated approachObject observations to update facing and beside fluents.
* segbot_logical_translator: fix broken add_dependency() (`#61 <https://github.com/utexas-bwi/segbot/issues/61>`_)
* improved door sensing behavior. switched from navfn to global_planner
* refined door open check behavior to also disable static costmap.
* changed gothrough action to disable static map while executing navigation.
* bug fixes to new floor location resolution functionality.
* added functionality for resolving a change floor request, allowing robot teleportation.
* Contributors: FernandezR, Jack O'Quin, Piyush Khandelwal

0.3.3 (2015-08-05)
------------------
* merge segbot_apps packages into segbot (`#46 <https://github.com/utexas-bwi/segbot/issues/46>`_)
* Contributors: Jack O'Quin

0.3.1 (2015-03-31)
------------------
* Closes `#29 <https://github.com/utexas-bwi/segbot_apps/issues/29>`_
  - Get latest version of move_base to get fix introduced in https://github.com/ros-planning/navigation/pull/295
  This allows setting tolerance to 0 when calling make_plan from segbot_logical_translator.
  - Don't use any tolerance when testing whether a door was open or not.
  - Don't clear costmap around robot when testing for an open door.
* fixed issue where the logical translator was returning beside and facing for two different doors. closes `#30 <https://github.com/utexas-bwi/segbot_apps/issues/30>`_.
* segbot_logical_navigator now checks for costmap updates to test whether the map has changed between door/no-door versions. closes `#28 <https://github.com/utexas-bwi/segbot_apps/issues/28>`_
* removed segbot_gazebo dependency. closes `#27 <https://github.com/utexas-bwi/segbot_apps/issues/27>`_.
* Contributors: Piyush Khandelwal

0.3.0 (2015-03-24)
------------------
* completed migration of LogicalNavigationAction to bwi_msgs in bwi_common repository.
* fixed bug in segbot_logical_translator where it would try and analyze facing/beside for doors not connected to the current location.
* modified the segbot_logical_navigator to use action service instead of services.
* read map file and data directory using multimap
* some navigation changes for more robust navigation.
* use correct approachable area for objects and doors.
* added multimap support.
* making the logical navigator advertise execute_logical_goal after the first pose has been received (github issue `#18 <https://github.com/utexas-bwi/segbot_apps/issues/18>`_)
* turning annoying prints into ROS_INFOs and removing more annoying ones.
* Made the door checker test for the door to be open 3 times before going through. Works much better. Removed recovery behavior.
* Contributors: Matteo Leonetti, Piyush Khandelwal

0.2.1 (2014-04-22)
------------------

0.2.0 (2014-04-19)
------------------
* now actually read in object approach file
* added the ability to approach an object, as well as added the
  ability to approach a door from any accessible location
* now catkin_lint approved
* better error checking, shared global nodehandle, added ability to
  sense whether a door is open or not
* moved the cost estimator to the bwi_planning package
* moved gazebo stuff to separate simulated app
* Contributors: Jack O'Quin, Piyush Khandelwal, piyushk

0.1.5 (2013-09-03)
------------------

0.1.4 (2013-08-12)
------------------

0.1.3 (2013-07-16)
------------------

0.1.2 (2013-07-13)
------------------

0.1.1 (2013-07-10)
------------------

0.1.0 (2013-06-28)
------------------
