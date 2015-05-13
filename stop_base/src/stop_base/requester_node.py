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

"""
.. module:: requester_node

Stop base controller requester node.

This Python module implements a requester interface for pausing or
running the robot base.

.. include:: weblinks.rst

"""

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import sys
import rospy

from .service import StopBaseClient
from .transitions import STATE_NAME


class RequesterNode(object):
    """ Stop base requester node.

    :param status: requested status.
    :type status: int
    """
    def __init__(self, status):
        """ Constructor. """
        rospy.init_node('stop_base_requester')
        self.client = StopBaseClient()

        # make the service request
        result = self.client.stop_base(
            status=status,
            requester=rospy.get_name())

        rospy.loginfo('Stop base is now '
                      + STATE_NAME[result.status.status])


STATUS_OPTIONS = {
        'RUNNING': 0, 'running': 0, 'r': 0,
        'PAUSED': 1, 'paused': 1, 'p': 1,
        'STOPPED': 2, 'stopped': 2, 's': 2}

def main():
    """ Requester node main entry point. """
    if len(sys.argv) < 2 or not sys.argv[1] in STATUS_OPTIONS:
        print(""" Usage:
rosrun stop_base request <status>

where: <status> can be:
        'RUNNING', 'running' or 'r'
        'PAUSED', 'paused' or 'p'
        'STOPPED', 'stopped' or 's'
""")
        sys.exit(9)

    node = RequesterNode(STATUS_OPTIONS[sys.argv[1]])
    sys.exit(0)
