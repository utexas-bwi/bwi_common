^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bwi_planning_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
