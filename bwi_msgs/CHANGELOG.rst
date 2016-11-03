^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bwi_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.12 (2016-09-14)
-------------------

0.3.11 (2016-08-27)
-------------------
* Added need_assist animation to LEDAnimations.msg in bwi_msgs (`#79
  <https://github.com/utexas-bwi/bwi_common/issues/79>`_)
* Contributors: Rolando Fernandez

0.3.10 (2016-08-15)
-------------------

0.3.9 (2016-08-05)
------------------
* added LED control messages, services and actions
* added a path to certificate messages from scav to virtour
* added a new scavenger hunt service that can pause/resume current task
* bwi_msgs: added a ScavStatus msg
* Contributors: Shiqi Zhang, Rolando Fernandez, Jack O'Quin

0.3.8 (2016-06-06)
------------------
* updated simulated calls that open the elevator doors to
  automatically close those doors.
* added a service interface to allow teleporting a robot.
* added service for resolving a change floor request.
* Contributors: Piyush Khandelwal

0.3.7 (2015-09-22)
------------------

0.3.6 (2015-08-25)
------------------
* add SemanticParser.srv to bwi_msgs (`#30 <https://github.com/utexas-bwi/bwi_common/issues/30>`_)
* Contributors: Jack O'Quin

0.3.5 (2015-08-22)
------------------
* bwi_msgs: fix catkin lint problems (`#31 <https://github.com/utexas-bwi/bwi_common/issues/31>`_)
* Contributors: Jack O'Quin

0.3.4 (2015-08-19)
------------------

0.3.3 (2015-08-05)
------------------
* stop_base: another unit test (`#23 <https://github.com/utexas-bwi/bwi_common/issues/23>`_, `utexas-bwi/segbot#41 <https://github.com/utexas-bwi/segbot/issues/41>`_)
* bwi_msgs: rename stop base service (`#23 <https://github.com/utexas-bwi/bwi_common/issues/23>`_, `utexas-bwi/segbot#41 <https://github.com/utexas-bwi/segbot/issues/41>`_)
* stop_base: initial stop base implementation (`#23 <https://github.com/utexas-bwi/bwi_common/issues/23>`_, `utexas-bwi/segbot#41 <https://github.com/utexas-bwi/segbot/issues/41>`_)
* bwi_msgs: add stop base controller interface (`utexas-bwi/segbot#41 <https://github.com/utexas-bwi/segbot/issues/41>`_)
* added a multi robot msg in preparation for multirobot experiments.
* Contributors: Jack O'Quin, Piyush Khandelwal

0.3.2 (2015-03-24)
------------------

0.3.1 (2015-03-16)
------------------

0.3.0 (2015-03-15)
------------------
* Initial release of package. It's purpose is to avoid circular dependencies due to messages. see `#15 <https://github.com/utexas-bwi/bwi_common/issues/15>`_.
* Contributors: Piyush Khandelwal
