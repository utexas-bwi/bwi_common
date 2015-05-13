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
.. module:: transitions

This module tracks stop base request state transitions.  The initial
state is RUNNING, and the terminal state is STOPPED.

.. graphviz:: state_transitions.dot

.. include:: weblinks.rst

"""

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

# Ros dependencies
import rospy

from bwi_msgs.msg import StopBaseStatus
from . import TransitionError

# Printable name for each state, indexed by number.
STATE_NAME = ['RUNNING', 'PAUSED', 'STOPPED']

## State transition table.
#
#  An immutable set of (old, new) status pairs.  All pairs in the
#  table are considered valid state transitions.  Any others are not.
#
TRANS_TABLE = frozenset([
    (StopBaseStatus.RUNNING, StopBaseStatus.RUNNING),
    (StopBaseStatus.RUNNING, StopBaseStatus.PAUSED),
    (StopBaseStatus.RUNNING, StopBaseStatus.STOPPED),

    (StopBaseStatus.PAUSED, StopBaseStatus.PAUSED),
    (StopBaseStatus.PAUSED, StopBaseStatus.RUNNING),
    (StopBaseStatus.PAUSED, StopBaseStatus.STOPPED),

    (StopBaseStatus.STOPPED, StopBaseStatus.STOPPED)])


class StopBaseState(object):
    """
    Class for tracking the status of a stop base request.

    .. describe:: str(status)

       :returns: String representation of this status.

    """
    def __init__(self):
        """ Constructor. """
        self.status = StopBaseStatus.RUNNING
        """ Current status. """
        self.pauses = set()
        """ Set of pause requester names. """

    def __str__(self):
        """ Generate string representation. """
        return str(STATE_NAME(self.status))

    def to_msg(self):
        """ :returns: corresponding bwi_msgs/StopBaseStatus message. """
        return StopBaseStatus(status=self.status)

    def transition(self, msg):
        """
        Update status based on this request message.

        :param msg: stop base service request message.
        :type msg: bwi_msgs/StopBaseRequest
        :raises: :exc:`.TransitionError` if not a valid transition.
        """
        if not self.valid(msg.status.status):
            raise TransitionError('invalid ' + STATE_NAME[msg.status.status]
                                  + ' request in state '
                                  + STATE_NAME[self.status])

        if msg.status.status == StopBaseStatus.PAUSED:
            if self.status != StopBaseStatus.STOPPED:
                self.pauses.add(msg.requester)
                self.status = msg.status.status
        elif msg.status.status == StopBaseStatus.RUNNING:
            if self.status != StopBaseStatus.STOPPED:
                self.pauses.discard(msg.requester)
                if self.status == StopBaseStatus.PAUSED:
                    if len(self.pauses) == 0:
                        self.status = msg.status.status
        elif msg.status.status == StopBaseStatus.STOPPED:
            self.status = msg.status.status

    def valid(self, new_status):
        """
        Valid state update predicate.

        :param new_status: Proposed new status for this request.
        :returns: ``True`` if this is a valid state transition.
        """
        return (self.status, new_status) in TRANS_TABLE
