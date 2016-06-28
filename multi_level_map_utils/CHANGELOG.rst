^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package multi_level_map_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2015-08-05)
------------------
* set queue_size for rospy Publishers (`#7 <https://github.com/utexas-bwi/multi_level_map/issues/7>`_)
* added a parameter for setting the default level. closes `#6 <https://github.com/utexas-bwi/multi_level_map/issues/6>`_.
* level_selector now waits for change_level service to be available before using it. closes `#5 <https://github.com/utexas-bwi/multi_level_map/issues/5>`_
* Contributors: Jack O'Quin, Piyush Khandelwal

0.1.1 (2015-03-24)
------------------
* removed unnecessary script installation which installs in global bin instead of package bin. closes `#4 <https://github.com/utexas-bwi/multi_level_map/issues/4>`_.
* Contributors: Piyush Khandelwal

0.1.0 (2015-03-14)
------------------
* cleaned up initial test release of the multi_level_map server now that it is being used within the BWI infrastructure.
* added a level selector and level mux utility to help change the current floor the robot is on.
* Contributors: Jack O'Quin, Piyush Khandelwal, piyushk
