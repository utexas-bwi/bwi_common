#! /usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (C) 2016, Jack O'Quin, Shiqi Zhang
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

""".. module:: distance

This Python module manages the directory into which logs are saved.
"""
# enable some python3 compatibility options:
from __future__ import absolute_import, print_function

import datetime
import os
import rosbag
import sys

def get_distance(filename):

    bag = rosbag.Bag(filename)
    time_start = -1
    distance_traveled = 0.0
    time_current = -1
    x = None
    y = None

    for topic, msg, t in bag.read_messages(topics=['odom_1hz']):

        time_current = int(msg.header.stamp.secs)
        x_current = float(msg.pose.pose.position.x)
        y_current = float(msg.pose.pose.position.y)

        if time_start < 0:
            x = x_current
            y = y_current
            time_start = time_current
            continue

        distance_traveled += ((x_current - x)**2 + (y_current - y)**2)**0.5
        x = x_current
        y = y_current

    bag.close()
    time_consumed = time_current - time_start
    return filename, time_consumed, distance_traveled


def main(argv=None):
    """ Main function (for testing). """
    if argv is None:
        argv = sys.argv
    if len(argv) != 2:
        print('error: no bag file name', file=sys.stderr)
        print("""
usage: rosrun bwi_logging distance.py filename
""", file=sys.stderr)
        return 9
    f, t, d = get_distance(argv[1])
    print('  filename: ' + f +
          '  time: ' + str(datetime.timedelta(seconds=t)) +
          '  distance: ' + str(d) + '\n')


if __name__ == '__main__':
    sys.exit(main())
