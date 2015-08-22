^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bwi_mapper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.5 (2015-08-22)
------------------

0.3.4 (2015-08-19)
------------------

0.3.3 (2015-08-05)
------------------
* added a script to translate graphs.
* some new graph functionality.
* Contributors: Piyush Khandelwal

0.3.2 (2015-03-24)
------------------

0.3.1 (2015-03-16)
------------------

0.3.0 (2015-03-15)
------------------
* fix many catkin lint errors
* indigo: fix opencv2 dependencies (`#10 <https://github.com/utexas-bwi/bwi_common/issues/10>`_)
* much improved implementation. probably still need to do better.
* added a path finder class for finding an obstacle free path in a map. It is now used by the logical navigation code
  to determine which approach point to use when approaching a door.
* updated the code to separate drawing an arrow in the image vs drawing an arrow using the graph
* enabled some globbing and made one of the cpp functions slightly faster.
* Contributors: Jack O'Quin, Piyush Khandelwal

0.2.4 (2014-04-29)
------------------
* cleaned up most catkin_lint warnings. closes `#6
  <https://github.com/utexas-bwi/bwi_common/issues/6>`_
* Added support for YAML-CPP 0.5+.  The new yaml-cpp API removes the
  "node >> outputvar;" operator, and it has a new way of loading
  documents.
* Contributors: Piyush Khandelwal, Scott K Logan

0.2.3 (2014-04-24)
------------------

0.2.2 (2014-04-19)
------------------

0.2.1 (2014-04-18)
------------------

0.2.0 (2014-04-16)
------------------

* Initial release to Hydro.
* Now passes catkin_lint checks.
* Added new image configuration files.
* Minor commits while improving the quality of images.
* Updated drawing functions a bit.
* Fixed bug in new convenience function call.
* Added convenience function. expand legend automatically.
* Added bwi_mapper to common components, as it is used by both
  planning and human guidance projects.
