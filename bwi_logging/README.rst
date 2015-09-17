Overview
========

The `bwi_logging`_ ROS_ package provides nodes and scripts for
collecting, analysing and uploading log data when running BWI robots.

ROS interface
=============

``record`` node
---------------

This ROS node is a wrapper for the standard `rosbag`_ ``record``
command, which it invokes after changing to an appropriate logging
directory.

The goal of this node is to choose a good place to save the bags.
When necessary, intermediate directories will be created, with group
write permissions, if possible.

Subscribed topics
'''''''''''''''''

All topics passed as command arguments will be subscribed by the
`rosbag`_ command, which this node launches.

Parameters
''''''''''

``~directory`` (string, default: ``~/.ros/bwi/bwi_logging``)
    An explicit directory for saving ROS topic bag files.

If the ``directory`` is not accessible, bags are saved in
``/tmp/bwi/bwi_logging``, instead.

Usage
'''''

::

    rosrun bwi_logging record topic1 [ topic2 ... ]

Where each ``topic`` is the name of a ROS topic to record.


``record`` launch script
------------------------

This ROS launch script runs the ``record`` node with appropriate
parameters.

Arguments
'''''''''

``directory`` (string, default: ``~/.ros/bwi/bwi_logging``)
    An explicit directory for saving ROS topic bag files.

``topics`` (string, default: ``odom amcl_pose /diagnostics``)
    The ROS topics to record.

Usage
'''''

To record the usual topics in the usual place::

    roslaunch bwi_logging record

To record different topic names::

    roslaunch bwi_logging record topics:='filtered_odom /diagnostics /tf'

To write the bag file in a different place::

    roslaunch bwi_logging record directory:="~bwilab/.ros/bwi/bwi_logging"


Scripts
=======

upload
------

Copies any newly-saved bag files in the current directory to the lab
server, optionally deleting the local copy afterwards. Only files with
names starting with ``bwi_`` and ending with ``.bag`` will be copied.

Files are stored in the ``~bwilab/host/$HOSTNAME`` directory on the
server, and ``$HOSTNAME`` should be set to the part of the full domain
name preceding the first dot.  Files already present on the server are
neither sent or deleted.

You may specify at most one of these options::

    -d  delete files uploaded successfully (default)
    -h  print a help message
    -k  keep uploaded files

Usage
'''''

To upload and then delete all new bag files for the current user::

    cd ~/.ros/bwi/bwi_logging
    rosrun bwi_logging upload

To upload and keep any new bag files in the current directory::

    rosrun bwi_logging upload -k

To force a specific host name::

    HOSTNAME=bender rosrun bwi_logging upload


.. _`bwi_logging`: http://wiki.ros.org/bwi_logging
.. _ROS: http:/ros.org
.. _`rosbag`: http://wiki.ros.org/rosbag
