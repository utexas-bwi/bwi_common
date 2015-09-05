#!/usr/bin/env python

""" Python unittest script for bwi_logging/directory module.

Does not require a ROS environment.
"""
# enable some python3 compatibility options:
from __future__ import absolute_import, print_function

PKG='bwi_logging'
import rospkg
import sys
import os
import stat
import unittest

from bwi_logging.directory import *

class TestLoggingDirectory(unittest.TestCase):
    """Unit tests for directory module. """

    def test_using_no_parameters(self):
        """Test with neither parameter provided."""
        home = os.environ['HOME']
        ld = LoggingDirectory(None, None)
        d = home + '/.ros/bwi/bwi_logging'
        self.assertEqual(ld.pwd(), d)
        ld.chdir()
        self.assertEqual(os.getcwd(), d)

    def test_invalid_account(self):
        """Test with invalid account name parameter."""
        acct = 'invalid_account'
        ld = LoggingDirectory(acct, None)
        d = '/tmp/bwi/bwi_logging'
        self.assertEqual(ld.pwd(), d)
        ld.chdir()
        self.assertEqual(os.getcwd(), d)

    def test_explicit_path_name(self):
        """Test with explicit path name parameter."""
        path = '/tmp/bwi_logging/tests'
        ld = LoggingDirectory(None, path)
        self.assertEqual(ld.pwd(), path)
        ld.chdir()
        self.assertEqual(os.getcwd(), path)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_camera_info_manager',
                    TestCameraInfoManager)
