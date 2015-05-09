#!/usr/bin/env python
""" cmd_vel requester for stop base controller tests. """

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import rospy
from geometry_msgs.msg import Twist


class CmdVelRequester(object):

    def __init__(self):
        """ Initialize ``cmd_vel`` topic requester. """
        rospy.init_node("cmd_vel_requester")
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.command = Twist()
        self.command.linear.x = 0.5
        self.update = 0.1
        self.timer = rospy.Timer(rospy.Duration(1.0), self.send_command)
        rospy.spin()

    def send_command(self, event):
        """ Timer event handler for periodically sending a command. """
        rospy.logdebug(str(self.command))
        self.pub_vel.publish(self.command)
        self.command.linear.x += self.update
        if self.command.linear.x >= 0.99 or self.command.linear.x <= 0.5:
            self.update = -self.update


if __name__ == '__main__':
    node = CmdVelRequester()
