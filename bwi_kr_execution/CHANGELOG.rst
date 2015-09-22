^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bwi_kr_execution
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.7 (2015-09-22)
------------------

0.3.6 (2015-08-25)
------------------
* bwi_kr_execution: remove segbot_gui references (`#30 <https://github.com/utexas-bwi/bwi_common/issues/30>`_)
* Contributors: Jack O'Quin

0.3.5 (2015-08-22)
------------------
* bwi_kr_execution: fix catkin lint warning (`#31 <https://github.com/utexas-bwi/bwi_common/issues/31>`_)
* Contributors: Jack O'Quin

0.3.4 (2015-08-19)
------------------

0.3.3 (2015-08-05)
------------------
* Indigo: change clingo dependency to gringo
* add a new node in bwi_kr_execution to keep updating states
* made current state query only return fluents at 0, perhaps safer for the learning executor
* upgrade to clingo 4.2.1 interface
* changed actasp to use clingo 4 syntax
* allowing AskPerson to work without the sound node
* adding a task for learning experiment, and a flag to the action theory to prevent the use of questions in tasks that don't need them
* adding time to the logging of actions and reward.
* Delete bwi_kr_execution_simulation.launch, closed issue `#20 <https://github.com/utexas-bwi/bwi_common/issues/20>`_
* Contributors: Jack O'Quin, Matteo Leonetti, Shiqi Zhang, yuqian

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
