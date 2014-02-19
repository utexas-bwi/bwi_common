#!/usr/bin/env python
#
# This code is based on code available at the following location: 
# https://www.assembla.com/code/XD_DSbot/subversion/nodes/trunk/Info/Simu/PololuMaestro.py?rev=405
# The original code is unlicensed, and has been re-packaged as BSD in this 
# code-release. Please contact the package distributors in case of licensing
# discrepancies
#
# Copyright (C) 2014, The Assembla Dev Team, Jack O'Quin, Piyush Khandelwal
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
This is a ROS device driver node that interacts with a Polulu Maestro Servo
Controller. It has been tested on the Polulu Micro MaestroDevice, but should
also work with the Polulu Mini MaestroDevice 12, 18 and 24.

.. note::

   TODO: add diagnositics

"""

import rospy
import serial
import time

class MaestroDevice:
    """ Class for managing the Polulu Maestro Servo Controller """

    def __init__(self, port='/dev/ttyACM0', baud=9600):
        self.port = port
        """ Path name for Arduino serial port. """
        self.baud = baud
        """ Baud rate for Arduino serial port. """
        self.dev = None
        """ Arduino serial device connection. """

    def close(self):
        if self.dev:
            self.dev.close()
        self.dev = None

    def ok(self):
        """ :returns: ``True`` if Arduino contacted. """
        return self.dev is not None

    def open(self):
        """ Open the Maestro serial device interface.

        :returns: ``True`` if open succeeds.
        """
        try:
            self.dev = serial.Serial(self.port, self.baud)
            # give a little time to the MaestroDevice to breeze 
            time.sleep(0.005)
        except IOError as e:
            # HACK: serial does not return errno.ENOTTY as it should,
            #       so check the exact string.
            enotty = ("Could not configure port: "
                      + "(25, 'Inappropriate ioctl for device')")
            if str(e) != enotty:        # is it a serial port?
                rospy.logerr('Serial port open failed at '
                             + str(self.baud) + ' baud: ' + str(e))
                return False
            else:
                rospy.logerr('Serial port is not configured as tty ' +
                             '(errno.ENOTTY): ' + str(e))
                return False
        else:
            rospy.loginfo('Serial port ' + self.port + ' opened at '
                          + str(self.baud) + ' baud.')
            self.dev.flushInput()       # discard any old data
            return True
  
    def __send_header(self, cmd, device):
        """ Sends the header of the command, depending ont the protocol """
        if  (device & 0x80) == 0 :              #Pololu protocol
            self.ser.write(chr(0xAA))           #start byte
            self.ser.write(chr(device & 0x7f))  #device
            cmd &= 0x7f                         #in this mode, the MSB of the 
        self.ser.write(chr(cmd))                #command is cleared

    def __send_command(self, cmd, device):
        """ Sends a command without data """
        self.__send_header(cmd, device)

    def __send_command_b(self, cmd, byte_arg, device):
        """ Sends a command with 1 byte of data """
        self.__send_header(cmd, device)
        self.ser.write(chr(byte_arg & 0x7f))

    def __send_command_bi(self, cmd, byte_arg, int_arg, device):
        """ sends a command with 3 byte of data """
        self.__send_header(cmd, device) 
        self.ser.write(chr(byte_arg & 0x7f))
        self.ser.write(chr(int_arg & 0x7f))        #data lower 7 bits
        self.ser.write(chr((int_arg >> 7) & 0x7f)) #data bits 7-13

    def __send_command_ii(self, cmd, int_arg1, int_arg2, device):
        """ sends a command with 4 bytes of data """
        self.__send_header(cmd, device) 
        self.ser.write(chr(int_arg1 & 0x7f))        #data lower 7 bits
        self.ser.write(chr((int_arg1 >> 7) & 0x7f)) #data bits 7-13
        self.ser.write(chr(int_arg2 & 0x7f))        #data lower 7 bits
        self.ser.write(chr((int_arg2 >> 7) & 0x7f)) #data bits 7-13

    def __send_command_var_i(self, cmd, byte_arg, int_values, device):
        """ Sends a command with variable bytes of data """
        self.__send_header(cmd, device)
        self.ser.write(chr(len(int_values)))     #number of int values
        self.ser.write(chr(byte_arg & 0x7f))     
        for i in int_values:
            self.ser.write(chr(i & 0x7f))        #data lower 7 bits
            self.ser.write(chr((i >> 7) & 0x7f)) #data bits 7-13

    def __receive_int(self):
        """ Receives 2 bytes of data """
        low  = self.ser.read()
        high = self.ser.read()
        return int(ord(low) | (ord(high) << 8))

    def __receive_byte(self):
        """ Receives 1 byte of data """
        return ord(self.ser.read())

    def set_target(self, servo, angle, device=0x80):
        """ Set the target for a servo
        :param servo: is the servo number (typically 0-23)
        :param angle: is the target, from 256 to 13120, in quarter-microseconds
                      (center=6000, off=0)
        :param device: is the id of the MaestroDevice device (0-127, default on
                       boards is 12, all device if not set)
        """
        self.__send_command_bi(0x84, servo, angle, device)

    def set_target_minissc(self,servo, angle):
        """ Set the target using the Mini-SSC protocol
        :param servo: is the sum of the servo number (typically 0-23) and the
                      servo offset of the device
        :param angle: is the target, from 0 to 254, mapped between
                      neutral-range and neutral+range. 127 is neutral.
        """
        self.ser.write("\xFF"+chr(servo)+chr(angle))

    def set_multiple_targets(self, first_servo, angles, device=0x80):
        """ Set simultaneously targets for a contiguous block of servos (Mini
        MaestroDevice 12, 18 and 24 only)
        :param first_servo: is the lower servo number in the block 
                            (typically 0-23)
        :param angles: is the list of targets, each from 256 to 13120, in
                       quarter-microseconds (center=6000, off=0)
        :param device: is the id of the MaestroDevice device (0-127, default on
                       boards is 12, all device if not set)
        """
        self.__send_command_var_i(0x9F, first_servo, angles, device)

    def set_speed(self, servo, speed, device=0x80):
        #set the speed for a servo
        :param servo: is the servo number (typically 0-23)
        :param speed: is the servo speed (0=full, 1=slower), in (0.25 us)/(10 ms) when T=20ms. see http://www.pololu.com/docs/0J40/4.e for details
        #device is the id of the MaestroDevice device (0-127, default on boards is 12, all device if not set)

        self.__send_command_bi(0x87, servo, speed, device)

    def set_acceleration(self, servo, acceleration, device=0x80):
        #set the acceleration for a servo
        #servo is the servo number (typically 0-23)
        #acceleration is servo acceleration (0=full, 1=slower), in (0.25 us)/(10 ms)/(80 ms) when T=20ms. see http://www.pololu.com/docs/0J40/4.e for details
        #device is the id of the MaestroDevice device (0-127, default on boards is 12, all device if not set)

        self.__send_command_bi(0x89, servo, acceleration, device)

    def set_pwm(self, on_time, period, device=0x80):
        #set the pwm output of a channel (Mini MaestroDevice 12, 18 and 24 only)
        #on_time is the part of the period the output will be high (unit : 1/48 us)
        #period is the total period length (unit : 1/48 us)
        # see http://www.pololu.com/docs/0J40/4.a for details
        self.__send_command_ii(0x8A, on_time, period, device)
    
    def go_home(self, device=0x80):
        #set all servos to their home position. Servos set to "ignore" will remain unchanged, servos set to "off" will turn off.
        #servo is the servo number (typically 0-23)
        #device is the id of the MaestroDevice device (0-127, default on boards is 12, all device if not set)

        self.__send_command(0xA2, device)

    def get_position(self, servo, device=0x80):
        #gets the current position of a servo
        #device is the id of the MaestroDevice device (0-127, default on boards is 12, all device if not set)
        
        self.__send_command_b(0x90, servo, device)
        return self.__receive_int()

    def get_moving_state(self, device=0x80):
        #checks if some servos are moving
        #device is the id of the MaestroDevice device (0-127, default on boards is 12, all device if not set)
        
        self.__send_command(0x93, device)
        return self.__receive_byte()

    def get_errors(self, device=0x80):
        #gets the current error flags
        #device is the id of the MaestroDevice device (0-127, default on boards is 12, all device if not set)
        
        self.__send_command(0xA1, device)
        return self.__receive_int()

from art_msgs.msg import SteeringCommand
from art_msgs.msg import SteeringState


class PololuDriver(object):
    def __init__(self):
        rospy.init_node('pololu_driver')
        rospy.on_shutdown(self.ShutdownCallback)
        rospy.Subscriber("steering/cmd",SteeringCommand, self.SteeringCallback)

        port = '/dev/ttyACM0'

        try:
            self.ser = serial.Serial(port)
            self.ser.open()
            self.ser.write(chr(0xAA))
            self.ser.flush()
        except serial.serialutil.SerialException as e:
            rospy.logerr(rospy.get_name()+": Error opening or initialising port "+port)
            exit(1)

    def DegToTicks(self,degrees):
        ticks = (max(-30,min(30,degrees))/30.0)*480
        return int(ticks + 1500)

    def run(self):
        rospy.spin()

    def get_command(self, channel, target):
        target = target * 4
        serialBytes = chr(0x84)+chr(channel)+chr(target & 0x7F)+chr((target >> 7) & 0x7F)
        return serialBytes

    def ShutdownCallback(self):
        rospy.loginfo("Shutting down")
        if hasattr(self, 'ser'):
            self.ser.write(self.get_command(0,0))
            self.ser.write(self.get_command(1,0))
            self.ser.write(chr(0xA2))
            self.ser.close()

    def SteeringCallback(self,data):
        ticks = self.DegToTicks(data.angle)
        rospy.loginfo(rospy.get_name()+": Request Type="+str(data.request)+" Angle="+str(data.angle)+" Ticks="+str(ticks) )
        ticks = self.DegToTicks(data.angle)
        self.ser.write(self.get_command(0,ticks))
        self.ser.write(self.get_command(1,ticks))


#Init and run
PololuDriver().run()
if __name__ == "__main__":
    ctrl = MaestroDevice('/dev/polulu_servo0')
    channel = 0
    ctrl.set_speed(channel,100)
    ctrl.set_target(channel,2000)
    print ctrl.get_position(channel)

#    for i in range(512, 512):
#        print i * 100
#	ctrl.set_target(channel,i*100)
#	time.sleep(0.1)
