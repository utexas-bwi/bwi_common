import copy
from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import QPoint, QRect, Qt
from python_qt_binding.QtGui import QFrame, QHBoxLayout, QImage, QLabel, QLineEdit, \
                                    QPainter, QPixmap, QPushButton, QColor, QTextBrowser, QVBoxLayout, \
                                    QWidget
# from python_qt_binding.QtCore import SIGNAL

def clear_layout(layout):
    # Clear all subfunction buttons.
    while layout.count():
        item = layout.takeAt(0)
        item.widget().deleteLater()

class MapImage(QLabel):

    def __init__(self, parent=None):
        super(MapImage, self).__init__(parent)

        # Image
        self.setFixedHeight(480)
        self.setFixedWidth(1080)
        self.setObjectName("map_image")

        # Set defaults to handle mouse events, as if these are not setup and a user clicks on the image, then all future
        # mouse events are ignored.
        self.mousePressEvent = self.defaultMouseHandler
        self.mouseMoveEvent = self.defaultMouseHandler
        self.mouseReleaseEvent = self.defaultMouseHandler

        # Create an image for the original map. This will never change.
        # TODO figure out how to update the map image, and how to read in the map file.
        map_image_location = "/home/piyushk/rocon/src/bwi_common/utexas_gdc/maps/3ne-real-new.pgm"
        map_image = QImage(map_image_location)
        self.map_image = map_image.scaled(1080, 480, Qt.KeepAspectRatio)

        # Create a pixmap for the overlay. This will be modified by functions to change what is being displayed 
        # on the screen.
        self.overlay_image = QImage(self.map_image.size(), QImage.Format_ARGB32) 
        self.overlay_image.fill(QColor(0,0,0,0))

        self.update()

    def defaultMouseHandler(self, event):
        # Do nothing.
        pass

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.drawImage(event.rect(), self.map_image, event.rect())
        painter.drawImage(event.rect(), self.overlay_image, event.rect())
        painter.end()

class LocationFunction(object):

    def __init__(self):
        self.subfunction_buttons = None
        self.default_subfunction = 'Edit Location Properties' 
        self.current_subfunction = self.default_subfunction

        # TODO read current locations from file.
        self.location_names = {}
        self.location_image = None
        self.current_selection_image = None

        self.is_modified = False

    def handleSubfunctionReset(self, 
                               widget,
                               subfunction_layout,
                               values_layout,
                               subfunction_button_callback,
                               image):

        if self.subfunction_buttons is not None:
            del self.subfunction_buttons[:]

        self.subfunction_buttons = []

        # Add all the necessary buttons to the subfunction layout.
        for button_text in ['Add Location', 'Remove Location', 'Edit Location']:
            button = QPushButton(button_text, widget)
            button.clicked[bool].connect(subfunction_button_callback)
            button.setCheckable(True)
            subfunction_layout.addWidget(button)
            self.subfunction_buttons.append(button)

        # When this function is engaged, 
        clear_layout(values_layout)

        self.image = image

        # Handle mouse event
        image.mousePressEvent = self.mousePressEvent
        image.mouseMoveEvent = self.mouseMoveEvent
        image.mouseReleaseEvent = self.mouseReleaseEvent

        self.start_point = None
        self.current_point = None

        if self.location_image is None:
            self.location_image = QImage(image.size(), QImage.Format_ARGB32)
            self.location_image.fill(QColor(0,0,0,0))

        if self.current_selection_image is None:
            self.current_selection_image = QImage(image.size(), QImage.Format_ARGB32)
            self.current_selection_image.fill(QColor(0,0,0,0))

        self.updateOverlay()

    def mousePressEvent(self, event):
        if self.start_point is not None:
            self.image.overlay_image.fill(QColor(0,0,0,0))
            self.image.update(self.get_rect(self.start_point, self.current_point))
        self.start_point = event.pos()
        self.current_point = event.pos()

    def mouseReleaseEvent(self, event):
        self.mouseMoveEvent(event)

    def mouseMoveEvent(self, event):

        # First make sure we update the region corresponding to the old mark.
        old_overlay_update_rect = self.get_rect(self.start_point, self.current_point)

        # Draw new mark.
        self.current_point = event.pos()
        painter = QPainter(self.current_selection_image)
        painter.setPen(QColor(255,0,0,128))
        painter.setBrush(QColor(255,0,0,128))
        self.new_selection = self.get_rect(self.start_point, self.current_point)
        draw_rect.setHeight(draw_rect.height() - 1)
        draw_rect.setWidth(draw_rect.width() - 1)
        painter.drawRect(draw_rect)

        painter.end()

        new_overlay_update_rect = self.get_rect(self.start_point, self.current_point)
        self.updateOverlay(self.get_max_rect(old_overlay_update_rect, new_overlay_update_rect))

    def get_rect(self, pt1, pt2):
        if pt1.x() < pt2.x():
            if pt1.y() < pt2.y():
                return QRect(pt1, pt2)
            else:
                return QRect(QPoint(pt1.x(), pt2.y()), QPoint(pt2.x(), pt1.y()))
        else:
            if pt1.y() < pt2.y():
                return QRect(QPoint(pt2.x(), pt1.y()), QPoint(pt1.x(), pt2.y()))
            else:
                return QRect(pt2, pt1)

    def get_max_rect(self, r1, r2):
        top_left_x = r1.topLeft().x() if (r1.topLeft().x() < r2.topLeft().x()) else r2.topLeft().x()
        top_left_y = r1.topLeft().y() if (r1.topLeft().y() < r2.topLeft().y()) else r2.topLeft().y()
        bottom_right_x = r1.bottomRight().x() if (r1.bottomRight().x() > r2.bottomRight().x()) else r2.bottomRight().x()
        bottom_right_y = r1.bottomRight().y() if (r1.bottomRight().y() > r2.bottomRight().y()) else r2.bottomRight().y()
        return QRect(QPoint(top_left_x, top_left_y), QPoint(bottom_right_x, bottom_right_y))

    def handleNewSubfunction(self,
                             widget,
                             source,
                             value_layout):

        # Ensure that all buttons apart from the current subfunction are depressed.
        for button in self.subfunction_buttons:
            if button != source:
                button.setChecked(False)

        # Switch to add/remove depending on the subfunction button pressed.
        self.current_subfunction = source.text()

    def updateOverlay(self, rect = None):
        # Redraw the overlay image from scratch using the location image and current location.
        self.image.overlay_image.fill(QColor(0,0,0,0))
        painter = QPainter(self.image.overlay_image)
        if self.location_image is not None:
            painter.drawImage(0, 0, self.location_image)
        if self.current_selection_image is not None:
            painter.drawImage(0, 0, self.current_selection_image)
        painter.end()

        if rect is None:
            self.image.update()
        else:
            self.image.update(rect)

class LogicalMarkerPlugin(Plugin):

    def __init__(self, context):
        super(LogicalMarkerPlugin, self).__init__(context)

        # TODO read in map file and data directory.
        # Give QObjects reasonable names
        self.setObjectName('LogicalMarkerPlugin')

        # Create QWidget
        self.master_widget = QWidget()
        self.master_layout = QVBoxLayout(self.master_widget)

        # Main Functions - Doors, Locations, Objects
        self.function_layout = QHBoxLayout()
        self.master_layout.addLayout(self.function_layout)
        self.function_buttons = []
        self.current_function = None
        for button_text in ['Locations']: #, 'Doors', 'Objects']:
            button = QPushButton(button_text, self.master_widget)
            button.clicked[bool].connect(self.handle_function_button)
            button.setCheckable(True)
            self.function_layout.addWidget(button)
            self.function_buttons.append(button)

        self.functions = {}
        self.functions['Locations'] = LocationFunction()

        self.master_layout.addWidget(self.get_horizontal_line())

        # Subfunction toolbar
        self.subfunction_layout = QHBoxLayout()
        self.master_layout.addLayout(self.subfunction_layout)
        self.current_subfunction = None

        self.master_layout.addWidget(self.get_horizontal_line())

        self.image = MapImage(self.master_widget)
        self.master_layout.addWidget(self.image)

        self.master_layout.addWidget(self.get_horizontal_line())

        # Configuration toolbar
        self.configuration_layout = QHBoxLayout()
        self.master_layout.addLayout(self.configuration_layout)

        self.master_widget.setObjectName('LogicalMarkerPluginUI')
        if context.serial_number() > 1:
            self.master_widget.setWindowTitle(self.master_widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self.master_widget)


    def construct_layout(self):
        pass

    def get_horizontal_line(self):
        """
        http://stackoverflow.com/questions/5671354/how-to-programmatically-make-a-horizontal-line-in-qt
        """
        hline = QFrame()
        hline.setFrameShape(QFrame.HLine)
        hline.setFrameShadow(QFrame.Sunken)
        return hline

    def handle_function_button(self):
        source = self.sender()

        if source.text() == self.current_function:
            source.setChecked(True)
            return

        # Depress all other buttons.
        for button in self.function_buttons:
            if button != source:
                button.setChecked(False)

        self.current_function = source.text()

        self.reset_subfunction()

    def reset_subfunction(self):

        # Clear all subfunction buttons.
        clear_layout(self.subfunction_layout)

        if self.current_function is not None:
            self.functions[self.current_function].handleSubfunctionReset(self.master_widget, 
                                                                         self.subfunction_layout, 
                                                                         self.configuration_layout,
                                                                         self.handle_subfunction_button,
                                                                         self.image)

        self.current_subfunction = None

    def handle_subfunction_button(self):
        source = self.sender()

        if source.text() == self.current_subfunction:
            source.setChecked(True)
            return

        self.functions[self.current_function].handleNewSubfunction(self.master_widget,
                                                                   source,
                                                                   self.configuration_layout)

        self.current_subfunction = source.text()

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
