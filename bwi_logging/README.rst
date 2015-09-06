Overview
========

The `bwi_logging`_ ROS_ package provides nodes and scripts for
collecting, analysing and uploading log data when running BWI robots.

ROS interface
=============

rosbag_record
-------------

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

``~account`` (string, default: "")
    A user account name on the local system for saving bag files (the
    current user, by default).  Unless an explicit ``directory``
    parameter was provided, save logs in the ``.ros/bwi/bwi_logging``
    subdirectory of the ``account`` home directory.

``~directory`` (string, default: "")
    An explicit directory for saving ROS topic bag files.  If
    ``directory`` is specified, the ``account`` parameter is ignored.

If none of the above locations are accessible, use
``/tmp/bwi/bwi_logging``.

Usage
'''''

::

    rosrun rosbag_record topic1 [ topic2 ... ]

Where each ``topic`` is the name of a ROS topic to record.

.. _`bwi_logging`: http://wiki.ros.org/bwi_logging
.. _ROS: http:/ros.org
.. _`rosbag`: http://wiki.ros.org/rosbag
