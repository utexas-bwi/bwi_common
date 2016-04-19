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

import os
import rospy
import subprocess
import sys

from .directory import LoggingDirectory

DEFAULT_PREFIX = 'bwi'


def main(argv=None):
    """ Main function. """

    # create node for reading ROS parameters
    rospy.init_node('record')

    # get the topics
    topics = rospy.get_param('~topics', None)
    if topics is None or topics == "":
        print('error: no topics to record', file=sys.stderr)
        print(' usage: rosrun bwi_logging record' +
              ' _topics:="topic1 topic2 ..."' +
              ' [_directory:=""] [_prefix:="bwi"]', file=sys.stderr)
        return 9
    rospy.loginfo('topics to record: ' + topics)

    # configure logging directory
    directory = rospy.get_param('~directory', None)
    rospy.loginfo('~directory: ' + str(directory))
    logdir = LoggingDirectory(directory)
    rospy.loginfo('logs go here: ' + logdir.pwd())
    logdir.chdir()                      # change to that directory

    # get the file prefix
    prefix = rospy.get_param('~prefix', DEFAULT_PREFIX)
    rospy.loginfo('logging with prefix: ' + str(prefix))

    # this is the command we will issue:
    cmd = ['rosbag', 'record', '-o' + prefix, '-j']
    cmd.extend(topics.split())
    rospy.loginfo('running command: ' + str(cmd))

    # run the rosbag command
    status = subprocess.call(cmd)

    rospy.loginfo('rosbag returned status: ' + str(status))

    if status == 0 and prefix == DEFAULT_PREFIX:

        rospy.loginfo('start uploading bags')

        # In the background, begin uploading the newly-written bag to
        # the BWI server.  The setsid isolates the uploading scripts
        # from ROS shutdown signals.  The -w120 waits two minutes
        # before starting to upload.
        upload_cmd = ['/usr/bin/setsid', '/usr/local/bin/bwi',
                      'bags', '-w120', '-d', logdir.pwd(), prefix, '&']
        cmd_str = ' '.join(x for x in upload_cmd)
        print('running command: ' + cmd_str)
        os.system(cmd_str)

    return status

if __name__ == '__main__':
    sys.exit(main())
