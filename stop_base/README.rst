Overview
========

The `stop_base`_ ROS_ package provides a controller node for pausing
and resuming robot motion by interrupting the ``/cmd_vel`` topic used
by move base and other navigation components.

ROS interface
=============

stop_base_controller
--------------------

This node provides a simple service interface for interrupting the
motion commands being passed to the robot base.

Subscribed topics
'''''''''''''''''

``cmd_vel`` (`geometry_msgs/Twist`_) 
    Robot movement requests.

Published topics
''''''''''''''''

``cmd_vel_safe`` (`geometry_msgs/Twist`_)
    Robot movement commands passed through unmodified if the
    controller is in the RUNNING state, otherwise the robot is
    commanded to stop.

``stop_base_status`` (`bwi_msgs/StopBaseStatus`_)
    The current status of this controller (latched topic).

Services
''''''''

``stop_base`` (`bwi_msgs/StopBase`_)
    ROS service interface for temporarily or permanently stopping
    robot base motion.

There can be multiple requesters, so the current status returned in
the response message may not match the original request, but that
request will remain active until it can be honored.  

If more than one requester sets a PAUSED status, the robot base will
remain motionless until they all request RUNNING again.  If any
requester sets STOPPED, no further requests will be met, the status
remaining permantently STOPPED.

Requester names should be sufficiently unique to avoid multiple
requesters picking the same string.  A reasonable choice is the ROS
node name.


Usage
'''''
::

    $ rosrun stop_base stop_base_controller


request node
------------

This node provides a command line interface for requesting
``stop_base_controller`` state transitions.

Usage
'''''

::

    rosrun stop_base request <status>

Where ``<status>`` can be:

 * ``RUNNING``, ``running`` or ``r``
 * ``PAUSED``, ``paused`` or ``p``
 * ``STOPPED``, ``stopped`` or ``s``

.. _`bwi_msgs/StopBase`:
   http://docs.ros.org/latest-available/api/bwi_msgs/html/srv/StopBase.html
.. _`bwi_msgs/StopBaseStatus`:
   http://docs.ros.org/latest-available/api/bwi_msgs/html/msg/StopBaseStatus.html
.. _`geometry_msgs/Twist`:
   http://docs.ros.org/latest-available/api/geometry_msgs/html/msg/Twist.html
.. _ROS: http:/ros.org
.. _`stop_base`: http://wiki.ros.org/stop_base
