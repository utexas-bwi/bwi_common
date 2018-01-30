#!/usr/bin/env python

from geometry_msgs.msg import PoseWithCovarianceStamped
from multi_level_map_msgs.msg import LevelMetaData, MultiLevelMapData
from multi_level_map_msgs.srv import ChangeCurrentLevel
import rospy

from python_qt_binding.QtCore import pyqtSignal
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import QLabel, QPushButton, QVBoxLayout, QWidget
from qt_gui.plugin import Plugin

from .utils import frameIdFromLevelId

class LevelSelectorPlugin(Plugin):

    button_signal = pyqtSignal()
    button_status_signal = pyqtSignal()

    def __init__(self, context):
        super(LevelSelectorPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('LevelSelectorPlugin')

        # Create QWidget
        self._widget = QWidget()
        self._widget.setFont(QFont("Times", 15, QFont.Bold))
        self._button_layout = QVBoxLayout(self._widget)

        self.buttons = []
        self.text_label = QLabel("Waiting for MultiLevelMapData...", self._widget)
        self._button_layout.addWidget(self.text_label)

        self._widget.setObjectName('LevelSelectorPluginUI')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle()
                                        + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        self.button_signal.connect(self.update_buttons)
        self.button_status_signal.connect(self.update_button_status)

        # Subscribe to the multi level map data to get information about all the maps.
        self.multimap_subscriber = rospy.Subscriber("map_metadata", MultiLevelMapData,
                                                    self.process_multimap)
        self.levels = []
        self.current_level = None
        rospy.loginfo('level selector: subscribed to maps')

        # Subscribe to the current level we are on.
        self.status_subscriber = None

        # Create a service proxy to change the current level.
        self.level_selector_proxy = rospy.ServiceProxy("level_mux/change_current_level",
                                                       ChangeCurrentLevel)
        self.level_selector_proxy.wait_for_service()
        rospy.loginfo('level selector: found "level_mux/change_current_level" service')

    def process_multimap(self, msg):
        """ Map metadata topic callback. """
        rospy.loginfo('level selector: map metadata received.')
        self.levels = msg.levels
        # update level buttons in the selection window
        self.button_signal.emit()

    def update_buttons(self):
        """ Update buttons in Qt window. """
        rospy.loginfo('level selector: update_buttons called')
        self.clean()
        for index, level in enumerate(self.levels):
            button = QPushButton(level.level_id, self._widget)
            button.clicked[bool].connect(self.handle_button)
            button.setCheckable(True)
            self._button_layout.addWidget(button)
            self.buttons.append(button)

        # Subscribe to the current level we are on.
        if self.status_subscriber is None:
            self.status_subscriber = rospy.Subscriber("level_mux/current_level",
                                                      LevelMetaData,
                                                      self.process_level_status)

    def update_button_status(self):
        """ Update button status Qt push button callback. """
        rospy.loginfo('level selector: update_button_status')
        if not self.buttons or not self.current_level:
            rospy.logwarn('level selector: current level not available')
            return
        rospy.logdebug('buttons: ' + str(self.buttons))
        for index, level in enumerate(self.levels):
            rospy.logdebug('level[' + str(index) + ']: ' + str(level.level_id))
            if self.current_level == level.level_id:
                self.buttons[index].setChecked(True)
            else:
                self.buttons[index].setChecked(False)

    def process_level_status(self, msg):
        """ level_mux/current_level topic callback. """
        rospy.loginfo('level selector: current_level topic message received')
        level_found = False
        for level in self.levels:
            if msg.level_id == level.level_id:
                self.current_level = level.level_id
                level_found = True
                break
        if not level_found:
            self.current_level = None
        self.button_status_signal.emit()

    def handle_button(self):
        source = self.sender()

        if source.text() == self.current_level:
            source.setChecked(True)
            return

        # Construct a identity pose. The level selector cannot be used
        # to choose the initialpose, as it does not have the interface
        # for specifying the position. The position should be
        # specified via rviz.
        origin_pose = PoseWithCovarianceStamped()
        origin_pose.header.frame_id = frameIdFromLevelId(source.text())
        origin_pose.pose.pose.orientation.w = 1    # Makes the origin quaternion valid.
        origin_pose.pose.covariance[0] = 1.0
        origin_pose.pose.covariance[7] = 1.0
        origin_pose.pose.covariance[14] = 1.0
        origin_pose.pose.covariance[21] = 1.0
        origin_pose.pose.covariance[28] = 1.0
        origin_pose.pose.covariance[35] = 1.0

        # Don't actually publish the initial pose via the level
        # selector. It doesn't know any better.
        self.level_selector_proxy(source.text(), False, origin_pose)

    def clean(self):
        while self._button_layout.count():
            item = self._button_layout.takeAt(0)
            item.widget().deleteLater()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
