#! /usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (C) 2015, Jack O'Quin
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the author nor of other contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

""".. module:: logger_node

This Python script runs `rosbag record` on a list of ROS topics,
saving the results in an appropriate directory for use with the BWI
robots.
"""
# enable some python3 compatibility options:
from __future__ import absolute_import, print_function

import sys
import rospy
from subprocess import call
#from .log_directory import change_dir

class UsageError(Exception):
    """ Usage exception for invalid arguments. """
    def __init__(self, msg):
        self.msg = msg


class LoggerNode(object):
    """ rosbag_record logger node. """
    def __init__(self, argv=None):
        """ Constructor. """
        if argv is None:
            argv = sys.argv
        if len(argv) < 2:
            raise UsageError('error: no topics to record')

        rospy.init_node('rosbag_record')
        account, directory = self.parameters()

        # this is the command we will issue
        print(str(['rosrun', 'rosbag', 'record'] + argv[1:]))
        rospy.sleep(2)

    def parameters(self):
        """ Get ROS parameter values.

        :returns: account, directory
        """
        account = rospy.get_param('~logging_account', None)
        rospy.loginfo('~logging_account: ' + str(account))
        directory = rospy.get_param('~logging_directory', None)
        rospy.loginfo('~logging_directory: ' + str(directory))
        return account, directory


def main(argv=None):
    """ Main function. """
    try:
        node = LoggerNode(argv)
    except UsageError, err:
        print(err.msg, file=sys.stderr)
        print("""
usage: rosbag_record topic1 [ topic2 ... ]
""", file=sys.stderr)
        return 9
    else:
        return 0


if __name__ == '__main__':
    sys.exit(main())
