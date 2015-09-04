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

Subscribed topics
'''''''''''''''''

All topics passed as command arguments will be subscribed by the
`rosbag`_ command, which this node launches.

Parameters
''''''''''

``~logging_account`` (string, default: "")
    The user account on the local system in which to save bag
    files. If none is specified, use ``${LOGNAME}`` in the current
    environment.

``~logging_directory`` (string, default: "")
    The desired directory for saving ROS topic bag files. If none is
    specified, use the ``.ros/bwi/logging`` subdirectory of the
    ``logging_account`` home directory. If that is not accessible, use
    ``/tmp/bwi/logging``.  When necessary and allowed, intermediate
    directories will be created, like with ``mkdir -p``.

Usage
'''''

::

    rosrun rosbag_record topic1 [ topic2 ... ]

Where each ``topic`` is the name of a ROS topic to record.

.. _`bwi_logging`: http://wiki.ros.org/bwi_logging
.. _ROS: http:/ros.org
.. _`rosbag`: http://wiki.ros.org/rosbag
