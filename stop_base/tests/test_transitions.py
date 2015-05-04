#!/usr/bin/env python

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

import copy
import uuid
import unittest

# ROS dependencies
from bwi_msgs.msg import StopBaseStatus

# module being tested:
from stop_base.transitions import *


class TestTransitions(unittest.TestCase):
    """Unit tests for stop base controller state transitions.

    These tests do not require a running ROS core.
    """

    ####################
    # utility methods
    ####################
    def assert_invalid(self, new_status, old_status, exception):
        """
        Assert that *new_status* with *old_status* fails, raising
        *exception*.
        """
        st = request_type(Request(id=unique_id.toMsg(TEST_UUID),
                                  resources=[TEST_WILDCARD],
                                  status=old_status))
        self.assertRaises(exception, st._transition(new_status))

    def assert_valid(self, request_type, old_status,
                     operation, new_status, *args):
        """
        Assert that *request_type* with *old_status* accepts named
        *operation*, yielding *new_status*.

        :returns: request contents after the *operation*.
        """
        rq = request_type(Request(id=unique_id.toMsg(TEST_UUID),
                                  resources=[TEST_WILDCARD],
                                  status=old_status))
        getattr(rq, operation)(*args)
        self.assertEqual(rq.msg.status, new_status)
        return rq

    ####################
    # request tests
    ####################
    def test_constructor(self):
        st = StopBaseState()
        self.assertEqual(st.status, StopBaseStatus.RUNNING)
        self.assertEqual(len(st.pauses), 0)

    def test_state_names(self):
        # test that STATE_NAME list matches the actual values
        self.assertEqual(len(STATE_NAME), 3)
        self.assertEqual(STATE_NAME[StopBaseStatus.RUNNING], 'RUNNING')
        self.assertEqual(STATE_NAME[StopBaseStatus.PAUSED], 'PAUSED')
        self.assertEqual(STATE_NAME[StopBaseStatus.STOPPED], 'STOPPED')

    def test_valid(self):
        st = StopBaseState()
        self.assertTrue(st._valid(StopBaseStatus.RUNNING))
        self.assertTrue(st._valid(StopBaseStatus.PAUSED))
        self.assertTrue(st._valid(StopBaseStatus.STOPPED))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('stop_base', 'test_transitions', TestTransitions)

