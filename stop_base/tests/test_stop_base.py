#!/usr/bin/env python
""" Requester for testing stop base controller. """

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import unittest
import rospy
import threading
from bwi_msgs.msg import StopBaseStatus
from bwi_msgs.srv import StopBase, StopBaseRequest, StopBaseResponse
from geometry_msgs.msg import Twist

from stop_base.service import StopBaseClient

class TestStopBase(unittest.TestCase):

    def test_stop_base_(self):
        """ Initialize requester for stop base service. """
        rospy.init_node("test_stop_base")
        self.lock = threading.RLock()
        self.node_name = rospy.get_name()
        self.expected_status = StopBaseStatus.RUNNING
        self.client = StopBaseClient()
        self.sub_vel = rospy.Subscriber('cmd_vel_safe', Twist,
                                        self.cmd_vel_callback)
        self.next_step = self.step1     # first step of test sequence
        self.timer = rospy.Timer(rospy.Duration(2.0), self.periodic_update)
        rospy.spin()

    def cmd_vel_callback(self, msg):
        """ Callback for cmd_vel_safe messages. """
        with self.lock:
            if self.expected_status == StopBaseStatus.RUNNING:
                self.assertGreaterEqual(msg.linear.x, 0.5)
            else:
                self.assertEqual(msg.linear.x, 0.0)

    def periodic_update(self, event):
        """ Timer event handler for periodic request updates.

        Invokes self.next_step(), unless ``None``.
        """
        if self.next_step is not None:  # more to do?
            self.next_step()
        else:                           # no more steps
            rospy.signal_shutdown('test completed.')

    def step1(self):
        rospy.loginfo('Step 1')
        self.next_step = self.step2

    def step2(self):
        rospy.loginfo('Step 2')
        with self.lock:
            self.expected_status = StopBaseStatus.PAUSED
            self.client.stop_base(
                status=self.expected_status,
                requester=self.node_name)
        self.next_step = self.step3

    def step3(self):
        rospy.loginfo('Step 3')
        with self.lock:
            self.expected_status = StopBaseStatus.RUNNING
            self.client.stop_base(
                status=self.expected_status,
                requester=self.node_name)
        self.next_step = self.step4

    def step4(self):
        rospy.loginfo('Step 4')
        self.next_step = None


if __name__ == '__main__':
    import rostest
    rostest.rosrun('stop_base', 'test_stop_base', TestStopBase)
