^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Common scripts for BWI Logical Planning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

logical_marker
--------------

This rqt plugin script is used to annotate a map. To run it, specify
the map's YAML file, and the directory for storing its logical
annotations.

For example::

  $ roscd utexas_gdc/maps/simulation/multimap2/3ne
  $ rosrun bwi_planning_common logical_marker _map_file:=3ne.yaml _data_directory:=.
