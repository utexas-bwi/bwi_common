# SimpleRobotSteeringPlugin plugin based on rqt_robot_steering. Original
# copyright notice below.
#
# Copyright (c) 2011, Dirk Thomas, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division

import math
import rospy
import time

from bwi_msgs.srv import QuestionDialog, QuestionDialogResponse, \
                         QuestionDialogRequest
from functools import partial
from qt_gui.plugin import Plugin

import os
import rospkg

from geometry_msgs.msg import Twist
from python_qt_binding import loadUi
from python_qt_binding.QtCore import pyqtSignal, Qt, QTimer, Slot
from python_qt_binding.QtGui import QFont
try:
    from python_qt_binding.QtWidgets import QGridLayout, QLabel, QLineEdit, QPushButton,\
                                            QTextBrowser, QVBoxLayout, QWidget
except ImportError:
    from python_qt_binding.QtGui import QGridLayout, QLabel, QLineEdit, QPushButton,\
                                        QTextBrowser, QVBoxLayout, QWidget

class QuestionDialogPlugin(Plugin):

    timeout_sig = pyqtSignal()
    update_sig = pyqtSignal()

    def __init__(self, context):
        super(QuestionDialogPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('QuestionDialogPlugin')

        font_size = rospy.get_param("~font_size", 40)

        # Create QWidget
        self._widget = QWidget()
        self._widget.setFont(QFont("Times", font_size, QFont.Bold))
        self._layout = QVBoxLayout(self._widget)
        self._text_browser = QTextBrowser(self._widget)
        self._layout.addWidget(self._text_browser)
        self._button_layout = QGridLayout()
        self._layout.addLayout(self._button_layout)

        # layout = QVBoxLayout(self._widget)
        # layout.addWidget(self.button)
        self._widget.setObjectName('QuestionDialogPluginUI')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        # Setup service provider
        self.service = rospy.Service('question_dialog', QuestionDialog,
                                     self.service_callback)
        self.request = None
        self.response_ready = False
        self.response = None
        self.buttons = []
        self.text_label = None
        self.text_input = None

        self.update_sig.connect(self.update)
        self.timeout_sig.connect(self.timeout)

    def shutdown_plugin(self):
        self.service.shutdown()

    def service_callback(self, req):
        self.response_ready = False
        self.request = req
        self.update_sig.emit()
        # Start timer against wall clock here instead of the ros clock.
        start_time = time.time()
        while not self.response_ready:
            if self.request != req:
                # The request got preempted by a new request.
                return QuestionDialogResponse(QuestionDialogRequest.PREEMPTED, "")
            if req.timeout != QuestionDialogRequest.NO_TIMEOUT:
                current_time = time.time()
                if current_time - start_time > req.timeout:
                    self.timeout_sig.emit()
                    return QuestionDialogResponse(
                            QuestionDialogRequest.TIMED_OUT, "")
            time.sleep(0.2)
        return self.response

    def update(self):
        self.clean()
        if not self.request:
            rospy.logwarn('question dialog: no request, update ignored')
            return
        req = self.request
        self._text_browser.setText(req.message)
        if req.type == QuestionDialogRequest.DISPLAY:
            # All done, nothing more too see here.
            self.response = QuestionDialogResponse(
                    QuestionDialogRequest.NO_RESPONSE, "")
            self.response_ready = True
        elif req.type == QuestionDialogRequest.CHOICE_QUESTION:
            for index, options in enumerate(req.options):
                button = QPushButton(options, self._widget)
                button.clicked.connect(partial(self.handle_button, index))
                row = index / 3
                col = index % 3
                self._button_layout.addWidget(button, row, col)
                self.buttons.append(button)
        elif req.type == QuestionDialogRequest.TEXT_QUESTION:
            self.text_label = QLabel("Enter here: ", self._widget)
            self._button_layout.addWidget(self.text_label, 0, 0)
            self.text_input = QLineEdit(self._widget)
            self.text_input.editingFinished.connect(self.handle_text)
            self._button_layout.addWidget(self.text_input, 0, 1)

    def timeout(self):
        self._text_browser.setText("Oh no! The request timed out.")
        self.clean()

    def clean(self):
        while self._button_layout.count():
            item = self._button_layout.takeAt(0)
            item.widget().deleteLater()
        self.buttons = []
        self.text_input = None
        self.text_label = None

    def handle_button(self, index):
        self.response = QuestionDialogResponse(index, "")
        self.clean()
        self.response_ready = True

    def handle_text(self):
        self.response = QuestionDialogResponse(
            QuestionDialogRequest.TEXT_RESPONSE,
            self.text_input.text())
        self.clean()
        self.response_ready = True

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

class SimpleRobotSteeringPlugin(Plugin):

    DEFAULT_LINEAR_VELOCITY = 1.0
    DEFAULT_ANGULAR_VELOCITY = math.pi / 2

    def __init__(self, context):
        super(SimpleRobotSteeringPlugin, self).__init__(context)
        self.setObjectName('SimpleRobotSteeringPlugin')

        self._publisher = None

        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('bwi_rqt_plugins'), 'resource', 'SimpleRobotSteering.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SimpleRobotSteeringUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        self._widget.keyPressEvent = self.keyPressEvent
        self._widget.keyReleaseEvent = self.keyReleaseEvent
        context.add_widget(self._widget)

        self._widget.topic_line_edit.textChanged.connect(self._on_topic_changed)

        self.linear_vel = 0
        self.angular_vel = 0

        # After doing so, key press events seem to work ok.
        self._widget.w_button.setFocus()

        # timer to consecutively send twist messages
        self._update_parameter_timer = QTimer(self)
        self._update_parameter_timer.timeout.connect(self._on_parameter_changed)
        self._update_parameter_timer.start(100)

    @Slot(str)
    def _on_topic_changed(self, topic):
        topic = str(topic)
        self._unregister_publisher()
        try:
            self._publisher = rospy.Publisher(topic, Twist, queue_size=10)
        except TypeError:
            self._publisher = rospy.Publisher(topic, Twist)

    def keyPressEvent(self, event):
        if not event.isAutoRepeat():
            if event.key() == Qt.Key_W:
                self.linear_vel = SimpleRobotSteeringPlugin.DEFAULT_LINEAR_VELOCITY
                self._widget.w_button.setDown(True)
                self._widget.s_button.setDown(False)
            elif event.key() == Qt.Key_S:
                self.linear_vel = -SimpleRobotSteeringPlugin.DEFAULT_LINEAR_VELOCITY
                self._widget.s_button.setDown(True)
                self._widget.w_button.setDown(False)
            elif event.key() == Qt.Key_A:
                self.angular_vel = SimpleRobotSteeringPlugin.DEFAULT_ANGULAR_VELOCITY
                self._widget.a_button.setDown(True)
                self._widget.d_button.setDown(False)
            elif event.key() == Qt.Key_D:
                self.angular_vel = -SimpleRobotSteeringPlugin.DEFAULT_ANGULAR_VELOCITY
                self._widget.d_button.setDown(True)
                self._widget.a_button.setDown(False)

    def keyReleaseEvent(self, event):
        if not event.isAutoRepeat():
            if self.linear_vel > 0 and event.key() == Qt.Key_W:
                self.linear_vel = 0
                self._widget.w_button.setDown(False)
            elif self.linear_vel < 0 and event.key() == Qt.Key_S:
                self.linear_vel = 0
                self._widget.s_button.setDown(False)
            elif self.angular_vel > 0 and event.key() == Qt.Key_A:
                self.angular_vel = 0
                self._widget.a_button.setDown(False)
            elif self.angular_vel < 0 and event.key() == Qt.Key_D:
                self.angular_vel = 0
                self._widget.d_button.setDown(False)


    def _on_parameter_changed(self):
        self._widget.linear_speed.setText("%.2f"%self.linear_vel)
        self._widget.angular_speed.setText("%.2f"%self.angular_vel)
        self._send_twist(self.linear_vel, self.angular_vel)

    def _send_twist(self, x_linear, z_angular):
        if self._publisher is None:
            return
        twist = Twist()
        twist.linear.x = x_linear
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = z_angular

        self._publisher.publish(twist)

    def _unregister_publisher(self):
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None

    def shutdown_plugin(self):
        self._update_parameter_timer.stop()
        self._unregister_publisher()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('topic' , self._widget.topic_line_edit.text())

    def restore_settings(self, plugin_settings, instance_settings):

        value = instance_settings.value('topic', "/cmd_vel")
        value = rospy.get_param("~default_topic", value)
        self._widget.topic_line_edit.setText(value)
