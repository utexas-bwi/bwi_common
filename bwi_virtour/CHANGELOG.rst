^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bwi_virtour
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.12 (2016-09-14)
-------------------
* set execute permissions when installing scripts (`utexas-bwi/bwi#46
  <https://github.com/utexas-bwi/bwi/issues/46>`_)
* document link to web page (`#78
  <https://github.com/utexas-bwi/bwi_common/issues/78>`_)
* add roslaunch check (`#73 <https://github.com/utexas-bwi/bwi_common/issues/73>`_)
* Contributors: Jack O'Quin

0.3.11 (2016-08-27)
-------------------

0.3.10 (2016-08-15)
-------------------
* bwi_virtour: install scripts in catkin bin destination (`#70 <https://github.com/utexas-bwi/bwi_common/issues/70>`_)
  collect scripts in sub-folder
  do not install web folder
* bwi_virtour: fix catkin_lint errors, missing installs (`#70 <https://github.com/utexas-bwi/bwi_common/issues/70>`_)
* changed the way robots are detected
* added notice if no robots available
* removed serve certificate from enable and added deliver_message
* cleaned up result handling
* Contributors: Jack O'Quin, Pato

0.3.9 (2016-08-05)
------------------
* New bwi_common package, reconcile version number (`#67
  <https://github.com/utexas-bwi/bwi_common/issues/67>`_)
* added starting kr execution and other service providers
* added launch files to enable and disable tours
* added UI for message delivery system
* added local debug version
* Contributors: Jack O'Quin, Pato
