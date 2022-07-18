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
.. module:: service

This Python module provides interfaces for StopBase service calls.

.. include:: weblinks.rst

"""
import rospy
from bwi_msgs.msg import StopBaseStatus
from bwi_msgs.srv import StopBase, StopBaseRequest, StopBaseResponse


def make_request(status, requester):
    """ Make a stop base service request.

    :param status: Requested status.
    :type status: int
    :param requester: Requester name.
    :type requester: str

    :returns: bwi_msgs/StopBaseRequest message.
    """
    return StopBaseRequest(
        status=StopBaseStatus(status=status),
        requester=requester)


def make_response(status):
    """ Make a stop base service response.

    :param status: Requested status.
    :type status: int

    :returns: bwi_msgs/StopBaseResponse message.
    """
    return StopBaseResponse(
        status=StopBaseStatus(status=status))


class StopBaseClient(object):
    """ Proxy class for making stop_base calls. """

    def __init__(self, srv_name='stop_base'):
        rospy.wait_for_service(srv_name)
        self.proxy = rospy.ServiceProxy(srv_name, StopBase)

    def stop_base(self, status, requester):
        """ Stop base service proxy.

        :param status: Requested status.
        :type status: int
        :param requester: Requester name.
        :type requester: str

        :returns: bwi_msgs/StopBaseResponse message.
        """
        return self.proxy(make_request(status, requester))
