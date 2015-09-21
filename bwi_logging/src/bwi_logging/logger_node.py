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

import rospy
import subprocess
import sys

from .directory import LoggingDirectory


def main(argv=None):
    """ Main function. """
    if argv is None:
        argv = sys.argv
    if len(argv) < 2:
        print('error: no topics to record', file=sys.stderr)
        print("""
usage: rosrun bwi_logging record topic1 [ topic2 ... ]
""", file=sys.stderr)
        return 9

    # create node for reading ROS parameters
    rospy.init_node('record')

    # configure logging directory
    directory = rospy.get_param('~directory', None)
    rospy.loginfo('~directory: ' + str(directory))
    logdir = LoggingDirectory(directory)
    rospy.loginfo('logs go here: ' + logdir.pwd())
    logdir.chdir()                      # change to that directory

    # this is the command we will issue:
    cmd = ['rosbag', 'record', '-obwi']
    cmd.extend(argv[1:])
    rospy.loginfo('running command: ' + str(cmd))

    return subprocess.call(cmd)


if __name__ == '__main__':
    sys.exit(main())
