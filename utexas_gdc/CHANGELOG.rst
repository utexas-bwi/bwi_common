^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package utexas_gdc
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.7 (2015-09-22)
------------------

0.3.6 (2015-08-25)
------------------

0.3.5 (2015-08-22)
------------------

0.3.4 (2015-08-19)
------------------

0.3.3 (2015-08-05)
------------------
* improved atrium mesh model.
* allow setting yaw for controllable person. also run atrium simulation in real time.
* added a simulation model for the 2nd floor.
* updated 3ne map.
* Contributors: Piyush Khandelwal

0.3.2 (2015-03-24)
------------------

0.3.1 (2015-03-16)
------------------

0.3.0 (2015-03-15)
------------------
* fix many catkin lint errors
  For remaining error, see `#13 <https://github.com/utexas-bwi/bwi_common/issues/13>`_
* started using the multimap infrastructure. This package now contains a lot of maps made using the updated tools. 
* Contributors: Jack O'Quin, Matteo Leonetti, Piyush Khandelwal, Shiqi Zhang

0.2.4 (2014-04-29)
------------------
* cleaned up most catkin_lint warnings. closes `#6
  <https://github.com/utexas-bwi/bwi_common/issues/6>`_
* Contributors: Piyush Khandelwal

0.2.3 (2014-04-24)
------------------

0.2.2 (2014-04-19)
------------------

0.2.1 (2014-04-18)
------------------
* Updated map PGM file to be obstacle free.

0.2.0 (2014-04-16)
------------------

* Initial release to Hydro.
* Move high-level launch scripts to bwi repo.
* Now passes catkin_lint checks.
* Fixed dependencies in package.xml, fixed greyscale color in pgm
  image, added a python mapper helper.
* Moved common research components from bwi_experimental to bwi_common
  repository in preparation of release
