^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bwi_kr_execution
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.8 (2016-06-06)
------------------
* fix broken clingo dependency (`#52 <https://github.com/utexas-bwi/bwi_common/issues/52>`_)
* moved disabling static map from bwi_kr_execution to
  segbot_logical_translator, where it should have been in the first
  place.
* the gothrough action disables the global costmap. the callelevator
  action only expects the door in front of it to open.
* added simulated change floor action. added hack to make plan shorter
  when using the elevator, and the other door opens.
* updated simulated calls that open the elevator doors to
  automatically close those doors.
* added a simulation change floor action. Should be ready for real
  robot testing now.
* added missing actions that won't be displayed in clingo output
* fixing planning issues with elevators
* some more updates to bwi_kr_execution. fixing some merge issues.
* merged with yuqian's branch.
* updated navigation facts for simulation.
* Added additional facts for 2nd floor to navigation facts.
* Changed location of elavator doors on 2nd floor to l2_200 from
  l2_302.
* Fixed static facts bug, by adding ``%#show hasdoor`` to
  elavator_facts. Fixed navigation bug on 2nd floor, by adding 2nd
  floor locations to navigation_facts.
* Clingo4 code fixes for current robot elevator code
* Contributors: Rolando Fernandez, Jack O'Quin, Piyush Khandelwal, Yuqian Jiang

0.3.2 (2015-03-24)
------------------
* updated Clingo to use the current state correctly.
* removed output that was placed in for debugging. closes `#17 <https://github.com/utexas-bwi/bwi_common/issues/17>`_.
* added missing directory installation. closes `#16 <https://github.com/utexas-bwi/bwi_common/issues/16>`_
* Contributors: Piyush Khandelwal

0.3.1 (2015-03-16)
------------------
* fixed broken link to costs.asp. costs are not being currently used.
* Contributors: Piyush Khandelwal

0.3.0 (2015-03-15)
------------------
* Initial release - migration from bwi_experimental into bwi_common.
* Contributors: Matteo Leonetti, Jack O'Quin, Piyush Khandelwal
