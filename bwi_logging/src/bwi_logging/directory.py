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

""".. module:: directory

This Python module manages the directory into which logs are saved.
"""
# enable some python3 compatibility options:
from __future__ import absolute_import, print_function

import os


class LoggingDirectory(object):
    """Logging directory object.

    :param directory: Directory path in which to save logs, or ``None``.
    """

    def __init__(self, directory):
        """ Constructor.

        :raises: :exc:`OSError` if no usable directory found.
        """
        try_dirs = []                 # list of directories to try
        if directory:
            try_dirs.append(directory)
        else:
            home = os.path.expanduser('~')
            if home[0] != '~':          # expansion successful?
                try_dirs.append(os.path.join(home, '.ros',
                                             'bwi', 'bwi_logging'))

        # /tmp/bwi/bwi_logging is the last resort if nothing else works
        try_dirs.append(os.path.join('/tmp', 'bwi', 'bwi_logging'))

        for d in try_dirs:
            if os.access(d, os.W_OK):       # is it writable?
                self.logdir = d
                return
            else:                           # create it, if possible
                um = os.umask(0o002)        # override group umask setting
                try:
                    os.makedirs(d, mode=0o2775)
                except OSError:
                    os.umask(um)            # restore umask
                    continue                # try another location
                else:
                    os.umask(um)            # restore umask
                    self.logdir = d
                    return

        # nothing worked
        raise OSError('error: no logging directory accessible')

    def chdir(self):
        """ Change current directory to the selected logging directory. """
        os.chdir(self.logdir)

    def pwd(self):
        """ :returns: selected logging directory. """
        return self.logdir
