^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package utexas_gdc
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.12 (2016-09-14)
-------------------

0.3.11 (2016-08-27)
-------------------

0.3.10 (2016-08-15)
-------------------
* fixed misnomer, changed default locations in go to location and visit door for testing, working
* changed approach points of the door
* added elevators
* added 4th floor map
* Contributors: Nathan John

0.3.9 (2016-08-05)
------------------
* Adjusted 1st floor door approach points.
* Changed Z position for second floor map to raise it above the first floor map.
* Created map for GDC 1st Floor and added new map to multimap.yaml
* Updated 3rd floor north approach points.
* Adjusted object o3_406_table to be inside area l3_406.
* Added south elevator to 2nd floor map
* Updated door approach points for 3rd floor map
* Updated door approach points and added objects to locations without doors.
* Removed annotations for doors without complete locations.
* Modified multimap.yaml to utilize to updated 3rd floor map.
* Added new complete map of 3rd floor, with additional doors and
  locations from south side of the building.
* Contributors: FernandezR

0.3.8 (2016-06-06)
------------------
* fixed problems with sketchup model.
* updated multimap.
* made more progress in migrating locations from the old map.
* added the updated simulation multimap
* door map for corpp experiments
* Contributors: Piyush Khandelwal, Shiqi Zhang

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
