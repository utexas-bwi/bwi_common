#!/usr/bin/env python

""" Python unittest script for bwi_logging/directory module.

Does not require a ROS environment.
"""
# enable some python3 compatibility options:
from __future__ import absolute_import, print_function

PKG='bwi_logging'
import os
import stat
import unittest

# module under test:
from bwi_logging.directory import *


def delete_file(filename):
    """ Delete a file, not complaining if it does not exist.
    :param filename: path to file.
    """
    try:
        os.remove(filename)
    except OSError:             # OK if file did not exist
        pass


class TestLoggingDirectory(unittest.TestCase):
    """Unit tests for directory module. """

    def test_using_no_parameters(self):
        """ Test with neither parameter provided. """
        home = os.environ['HOME']
        ld = LoggingDirectory(None)
        d = home + '/.ros/bwi/bwi_logging'
        self.assertEqual(ld.pwd(), d)
        ld.chdir()
        self.assertEqual(os.getcwd(), d)

    def test_explicit_path_name(self):
        """ Test with explicit path name parameter. """
        um = os.umask(0o022)            # start by suppressing group write
        parent = '/tmp/bwi_logging'
        os.system('rm -rf ' + parent)

        # make sure at least two directories are created
        path = os.path.join(parent, 'tests')
        ld = LoggingDirectory(path)
        self.assertEqual(ld.pwd(), path)
        ld.chdir()
        self.assertEqual(os.getcwd(), path)

        # verify group write permissions
        st = os.stat(path)
        self.assertEqual(st.st_mode & stat.S_IWGRP, stat.S_IWGRP)
        os.umask(um)                    # restore umask permissions


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_camera_info_manager',
                    TestCameraInfoManager)
