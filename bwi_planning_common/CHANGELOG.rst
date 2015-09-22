^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bwi_planning_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

0.3.2 (2015-03-24)
------------------
* removed accidental global script installation. closes `#18 <https://github.com/utexas-bwi/bwi_common/issues/18>`_.
* Contributors: Piyush Khandelwal

0.3.1 (2015-03-16)
------------------

0.3.0 (2015-03-15)
------------------
* bwi_planning_common: remove run_depend on opencv2
  Also, converted the package.xml to format two.  Fixes `#15 <https://github.com/utexas-bwi/bwi_common/issues/15>`_.
* fix many catkin lint errors
  For remaining error, see `#13 <https://github.com/utexas-bwi/bwi_common/issues/13>`_
* indigo: fix opencv2 dependencies (`#10 <https://github.com/utexas-bwi/bwi_common/issues/10>`_)
* created a new tool for marking logical locations, and overhauled how logical information was being stored.
  added map for the atrium
* Fixed a hack where each door needed the entire list of locations from which the door was directly accessible, instead
  of just the adjacent location.
* Contributors: Jack O'Quin, Matteo Leonetti, Piyush Khandelwal

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
* Install header correctly
  (`#1 <https://github.com/utexas-bwi/bwi_common/issues/1>`_)

0.2.1 (2014-04-18)
------------------

* Initial release to Hydro.
* Add roslaunch file checks.  This required providing default values
  for the ``map_file`` arguments.
* Install launch files.
* Move ``bwi_planning_common`` to ``bwi_common`` from the
  ``bwi_planning`` repository (`bwi_planning#1`_).
* Updating bwi_planning code for the ICAPS 2014 paper.
* Added resolveDoor function.  Added ability to add cost samples using
  a service in the node.
* Added planning common (formerly segbot_common).

.. _`bwi_planning#1`: https://github.com/utexas-bwi/bwi_planning/issues/1
