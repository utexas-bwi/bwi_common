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

""".. module:: bagfile

This Python module provides function for accessing information saved
in ROS bag files.

"""
# enable some python3 compatibility options:
from __future__ import absolute_import, print_function

import os
import rosbag

def distance_traveled(filename):
    """Compute distance traveled and time consumed.

    :param: filename path to bag file.
    :type:  str

    :returns: (time_consumed, distance_traveled, filename) tuple.
              The time is in seconds, distance in meters.

    Reports zero time and distance if filename is not a valid bag file.
    """

    try:
        bag = rosbag.Bag(filename)
    except IOError:
        return 0, 0.0, filename

    time_start = -1
    total_distance = 0.0
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

        total_distance += ((x_current - x)**2 + (y_current - y)**2)**0.5
        x = x_current
        y = y_current

    bag.close()
    time_consumed = time_current - time_start
    return time_consumed, total_distance, filename
