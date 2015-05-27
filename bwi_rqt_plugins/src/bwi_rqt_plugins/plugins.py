import rospy
import time

from bwi_msgs.srv import QuestionDialog, QuestionDialogResponse, \
                         QuestionDialogRequest
from functools import partial
from qt_gui.plugin import Plugin
from python_qt_binding.QtGui import QFont, QHBoxLayout, QLabel, QLineEdit, \
                                    QPushButton, QTextBrowser, QVBoxLayout, \
                                    QWidget
from python_qt_binding.QtCore import SIGNAL

class QuestionDialogPlugin(Plugin):

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
        self._button_layout = QHBoxLayout()
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
        self.response_ready = False
        self.response = None
        self.buttons = []
        self.text_label = None
        self.text_input = None

        self.connect(self._widget, SIGNAL("update"), self.update)
        self.connect(self._widget, SIGNAL("timeout"), self.timeout)

    def shutdown_plugin(self):
        self.service.shutdown()

    def service_callback(self, req):
        self.response_ready = False
        self.request = req
        self._widget.emit(SIGNAL("update"))
        # Start timer against wall clock here instead of the ros clock.
        start_time = time.time()
        while not self.response_ready:
            if self.request != req:
                # The request got preempted by a new request.
                return QuestionDialogResponse(QuestionDialogRequest.PREEMPTED, "")
            if req.timeout != QuestionDialogRequest.NO_TIMEOUT:
                current_time = time.time()
                if current_time - start_time > req.timeout:
                    self._widget.emit(SIGNAL("timeout"))
                    return QuestionDialogResponse(
                            QuestionDialogRequest.TIMED_OUT, "")
            time.sleep(0.2)
        return self.response

    def update(self):
        self.clean()
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
                self._button_layout.addWidget(button)
                self.buttons.append(button)
        elif req.type == QuestionDialogRequest.TEXT_QUESTION:
            self.text_label = QLabel("Enter here: ", self._widget)
            self._button_layout.addWidget(self.text_label)
            self.text_input = QLineEdit(self._widget)
            self.text_input.editingFinished.connect(self.handle_text)
            self._button_layout.addWidget(self.text_input)

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
