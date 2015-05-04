#!/usr/bin/env python
""" Requester for testing stop base controller. """

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import unittest
import rospy
from bwi_msgs.msg import StopBaseRequest


class TestStopBase(unittest.TestCase):

    def test_stop_base_(self):
        """ Initialize ROCON scheduler node for example requester. """
        rospy.init_node("test_stop_base")
        self.rqr = Requester(self.feedback, frequency=1.0)
        self.next_step = self.step1     # first step of test sequence
        self.timer = rospy.Timer(rospy.Duration(2.0), self.periodic_update)
        rospy.spin()

    def feedback(self, rset):
        """ Scheduler feedback function. """
        rospy.loginfo('feedback callback:')
        for rq in rset.values():
            rospy.logdebug('  ' + str(rq))
            if rq.msg.status == Request.WAITING:
                rospy.loginfo('  request queued: ' + str(rq.uuid))
            elif rq.msg.status == Request.GRANTED:
                rospy.loginfo('  request granted: ' + str(rq.uuid))
            elif rq.msg.status == Request.CLOSED:
                rospy.loginfo('  request closed: ' + str(rq.uuid))
            elif rq.msg.status == Request.PREEMPTING:
                rospy.loginfo('  request preempted (reason='
                              + str(rq.msg.reason) + '): ' + str(rq.uuid))
                rq.cancel()     # release preempted resources immediately

    def periodic_update(self, event):
        """ Timer event handler for periodic request updates.

        Invokes self.next_step(), unless ``None``.
        """
        if self.next_step is not None:  # more to do?
            self.next_step()
        else:                           # no more steps
            rospy.signal_shutdown('test completed.')

    def request_turtlebot(self):
        """ Request any tutlebot able to run *example_rapp*.

        :returns: UUID of new request sent.
        """
        bot = Resource(rapp='tests/example_rapp', uri='rocon:/turtlebot')
        rq_id = self.rqr.new_request([bot])
        rospy.loginfo('  new request: ' + str(rq_id))
        return rq_id

    def verify(self, rq_list):
        self.assertEqual(len(self.rqr.rset), len(rq_list))
        for rq in rq_list:
            self.assertTrue(rq in self.rqr.rset)


if __name__ == '__main__':
    import rostest
    rostest.rosrun('stop_base', 'test_stop_base', TestStopBase)
