from functools import partial
from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import QPoint, Qt
from python_qt_binding.QtGui import QFrame, QHBoxLayout, QImage, QLabel, QLayout, QLineEdit, \
                                    QPainter, QPolygon, QPushButton, QColor, QVBoxLayout, \
                                    QWidget
# from python_qt_binding.QtCore import SIGNAL

def clearLayoutAndFixHeight(layout):
    # Clear all subfunction buttons.
    while layout.count():
        item = layout.takeAt(0)
        if item.widget() is not None:
            item.widget().deleteLater()
    height_label = QLabel()
    height_label.setFixedHeight(30)
    layout.addWidget(height_label)

class MapImage(QLabel):

    def __init__(self, parent=None):
        super(MapImage, self).__init__(parent)

        # Image
        self.setFixedHeight(480)
        self.setFixedWidth(1080)
        self.setObjectName("map_image")

        # Set defaults to handle mouse events, as if these are not setup and a user clicks on the image, then all future
        # mouse events are ignored.
        self.enableDefaultMouseHooks()

        # Create an image for the original map. This will never change.
        # TODO read map from ROS param.
        map_image_location = "/home/piyushk/rocon/src/bwi_common/utexas_gdc/maps/3ne-real-new.pgm"
        map_image = QImage(map_image_location)
        self.map_image = map_image.scaled(1080, 480, Qt.KeepAspectRatio)

        # Create a pixmap for the overlay. This will be modified by functions to change what is being displayed 
        # on the screen.
        self.overlay_image = QImage(self.map_image.size(), QImage.Format_ARGB32) 
        self.overlay_image.fill(Qt.transparent)

        self.update()

    def defaultMouseHandler(self, event):
        # Do nothing.
        pass

    def enableDefaultMouseHooks(self):
        self.mousePressEvent = self.defaultMouseHandler
        self.mouseMoveEvent = self.defaultMouseHandler
        self.mouseReleaseEvent = self.defaultMouseHandler

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.drawImage(event.rect(), self.map_image, event.rect())
        painter.drawImage(event.rect(), self.overlay_image, event.rect())
        painter.end()

class LocationFunction(object):

    EDIT_LOCATION_PROPERITIES = 'Edit Location Properties'
    ADD_LOCATION = 'Add Location'
    EDIT_LOCATION = 'Edit Location'

    def __init__(self,
                 widget,
                 subfunction_layout,
                 configuration_layout,
                 image):
        self.subfunction_buttons = None
        self.default_subfunction = LocationFunction.EDIT_LOCATION_PROPERITIES
        self.current_subfunction = self.default_subfunction

        self.current_selection_color = Qt.black

        # Dictionary of polygons
        # TODO read from file
        self.locations = {}
        # Dictionary that maps location names to their colors
        self.location_colors = {}
        self.draw_location = {}
        self.new_loc_counter = 1

        self.currently_selected_location = None

        self.is_modified = False

        self.widget = widget
        self.subfunction_layout = subfunction_layout
        self.image = image
        self.configuration_layout = configuration_layout

        self.subfunction_buttons = {}

    def deactivateFunction(self):

        clearLayoutAndFixHeight(self.subfunction_layout)
        self.subfunction_buttons.clear()
        self.image.enableDefaultMouseHooks()

        self.clearActiveLocation()
        self.updateCurrentLocation(None)

        # Just in case we were editing a location, that location was not being drawn. 
        for location in self.draw_location:
            self.draw_location[location] = True

        self.changeSubfunctionTo(self.default_subfunction)

    def activateFunction(self):

        # Add all the necessary buttons to the subfunction layout.
        clearLayoutAndFixHeight(self.subfunction_layout)
        for button_text in [LocationFunction.ADD_LOCATION, 
                            LocationFunction.EDIT_LOCATION]:
            button = QPushButton(button_text, self.widget)
            button.clicked[bool].connect(partial(self.changeSubfunctionTo, button_text))
            button.setCheckable(True)
            self.subfunction_layout.addWidget(button)
            self.subfunction_buttons[button_text] = button
        self.subfunction_layout.addStretch(1)

        # ActivateMouseHooks.
        self.image.mousePressEvent = self.mousePressEvent
        self.image.mouseMoveEvent = self.mouseMoveEvent
        self.image.mouseReleaseEvent = self.mouseReleaseEvent

        self.clearActiveLocation()
        self.updateCurrentLocation(None)

        self.updateOverlay()

    def clearActiveLocation(self):

        # Make sure all selections are clear.
        self.new_selection_start_point = None
        self.new_selection_end_point = None

        # QPolygons to track current location.
        self.current_selection = None
        self.new_selection = None
        self.subtract_new_selection = None

        self.updateOverlay()

    def changeSubfunctionTo(self, source_text, currently_selected_location = None):

        if source_text == self.current_subfunction:
            # Nothing to see here. Move along.
            self.subfunction_buttons[source_text].setChecked(True)
            return

        # Ensure that all buttons apart from the current subfunction are depressed.
        for button in self.subfunction_buttons:
            if button != source_text:
                self.subfunction_buttons[button].setChecked(False)

        # Switch to add/remove depending on the subfunction button pressed.
        self.current_subfunction = source_text

        # Reset any selections.
        self.clearActiveLocation()

        if ((self.current_subfunction == LocationFunction.ADD_LOCATION) or
            (self.current_subfunction == LocationFunction.EDIT_LOCATION)):
            clearLayoutAndFixHeight(self.configuration_layout)
            for button_text in ["Done", "Cancel"]:
                button = QPushButton(button_text, self.widget)
                button.clicked[bool].connect(partial(self.finish_location_edit, button_text))
                self.configuration_layout.addWidget(button)
            self.configuration_layout.addStretch(1)

        if (self.current_subfunction == LocationFunction.EDIT_LOCATION):
            self.draw_location[self.currently_selected_location] = False 
            self.current_selection = self.locations[self.currently_selected_location]
            self.editing_location = self.currently_selected_location

        self.updateCurrentLocation(currently_selected_location)
        print "asdf"
        self.updateOverlay()

    def finish_location_edit(self, button_text):
        select_location = None
        if button_text == "Done" and self.current_selection is not None:
            for location in self.locations:
                # Before adding this location, remove from all other locations any portion that is now part of this one.
                self.locations[location] = self.locations[location].subtracted(self.current_selection) 

            if self.current_subfunction == LocationFunction.ADD_LOCATION:
                new_location_name = self.generate_new_location_id()
                self.locations[new_location_name] = self.current_selection
                self.location_colors[new_location_name] = self.getUniqueColor()
                self.draw_location[new_location_name] = True
                self.new_loc_counter += 1
                select_location = new_location_name
            else:
                self.locations[self.editing_location] = self.current_selection
                self.draw_location[self.editing_location] = True
                select_location = self.editing_location
            
            self.check_all_locations()

        self.clearActiveLocation()
        self.changeSubfunctionTo(LocationFunction.EDIT_LOCATION_PROPERITIES, select_location)

    def check_all_locations(self):
        # TODO Remove null locations. Merge two locations with the same name and remove the redundant one.
        pass

    def name_textedit_updated(self, text):
        if text != self.currently_selected_location:
            self.update_name_button.setEnabled(True)
        else:
            self.update_name_button.setEnabled(False)

    def update_location_name(self):
        old_loc_name = self.currently_selected_location
        new_loc_name = self.update_name_textedit.text()
        self.locations[new_loc_name] = self.locations.pop(old_loc_name)
        self.location_colors[new_loc_name] = self.location_colors.pop(old_loc_name)
        self.draw_location[new_loc_name] = self.draw_location.pop(old_loc_name)
        self.updateCurrentLocation(new_loc_name)

    def remove_current_location(self):
        old_loc_name = self.currently_selected_location
        self.locations.pop(old_loc_name)
        self.location_colors.pop(old_loc_name)
        self.draw_location.pop(old_loc_name)
        self.updateCurrentLocation(None)
        self.updateOverlay()

    def generate_new_location_id(self):
        return "new_loc" + str(self.new_loc_counter)

    def isModified(self):
        return self.is_modified

    def updateCurrentLocation(self, loc):
        if self.current_subfunction == LocationFunction.EDIT_LOCATION_PROPERITIES:
            self.currently_selected_location = loc 
            if loc is not None:
                self.subfunction_buttons[LocationFunction.EDIT_LOCATION].setEnabled(True)

                clearLayoutAndFixHeight(self.configuration_layout)

                self.update_name_label = QLabel("Location (" + self.currently_selected_location + ")      New Name: ", self.widget)
                self.configuration_layout.addWidget(self.update_name_label)

                self.update_name_textedit = QLineEdit(self.widget)
                self.update_name_textedit.setText(loc)
                self.update_name_textedit.textEdited.connect(self.name_textedit_updated)
                self.configuration_layout.addWidget(self.update_name_textedit)

                self.update_name_button = QPushButton("Update location Name", self.widget)
                self.update_name_button.clicked[bool].connect(self.update_location_name)
                self.update_name_button.setEnabled(False)
                self.configuration_layout.addWidget(self.update_name_button)

                self.remove_location_button = QPushButton("Remove Location", self.widget)
                self.remove_location_button.clicked[bool].connect(self.remove_current_location)
                self.configuration_layout.addWidget(self.remove_location_button)

                self.configuration_layout.addStretch(1)
            else:
                self.subfunction_buttons[LocationFunction.EDIT_LOCATION].setEnabled(False)
                clearLayoutAndFixHeight(self.configuration_layout)
                self.update_name_label = None
                self.update_name_textedit = None
                self.update_name_button = None
        self.updateOverlay()

    def mousePressEvent(self, event):
        if ((self.current_subfunction == LocationFunction.ADD_LOCATION) or
            (self.current_subfunction == LocationFunction.EDIT_LOCATION)):
            self.subtract_new_selection = event.button() == Qt.RightButton
            self.new_selection_start_point = event.pos()
            self.new_selection_end_point = event.pos()
            self.new_selection = None
        elif (self.current_subfunction == LocationFunction.EDIT_LOCATION_PROPERITIES):
            loc = self.getLocationNameFromPoint(event.pos()) 
            self.updateCurrentLocation(loc)

    def mouseReleaseEvent(self, event):
        if ((self.current_subfunction == LocationFunction.ADD_LOCATION) or
            (self.current_subfunction == LocationFunction.EDIT_LOCATION)):
            self.mouseMoveEvent(event)
            if self.new_selection is not None:
                if self.current_selection is None and self.subtract_new_selection == False:
                    self.current_selection = self.new_selection
                if self.subtract_new_selection:
                    self.current_selection = self.current_selection.subtracted(self.new_selection)
                else:
                    self.current_selection = self.current_selection.united(self.new_selection)
            self.new_selection = None
            self.subtract_new_selection = None

    def mouseMoveEvent(self, event):

        if ((self.current_subfunction == LocationFunction.ADD_LOCATION) or
            (self.current_subfunction == LocationFunction.EDIT_LOCATION)):

            # First make sure we update the region corresponding to the old mark.
            old_overlay_update_rect = self.get_rectangular_polygon(self.new_selection_start_point, self.new_selection_end_point)

            # Draw new mark, taking some care to reduce the size of the polygon's bottom right corner by (1,1).
            self.new_selection_end_point = event.pos()
            self.new_selection = self.get_rectangular_polygon(self.new_selection_start_point, self.new_selection_end_point)
            self.new_selection = self.new_selection.boundingRect()
            self.new_selection.setHeight(self.new_selection.height() - 1)
            self.new_selection.setWidth(self.new_selection.width() - 1)
            self.new_selection = QPolygon(self.new_selection, True)

            # Next determine the region that needs to be update because of the new mark.
            new_overlay_update_rect = self.get_rectangular_polygon(self.new_selection_start_point, self.new_selection_end_point)

            overlay_update_region = (old_overlay_update_rect + new_overlay_update_rect).boundingRect()
            self.updateOverlay(overlay_update_region)

    def updateOverlay(self, rect = None):
        # Redraw the overlay image from scratch using the location image and current location.

        self.image.overlay_image.fill(Qt.transparent)
        painter = QPainter(self.image.overlay_image)
        painter.setBackgroundMode(Qt.TransparentMode)
        painter.setCompositionMode(QPainter.CompositionMode_Source)

        for location in self.locations:
            if self.draw_location[location]:
                color = self.location_colors[location]
                if self.currently_selected_location == location:
                    color = self.current_selection_color
                lineColor = QColor(color)
                lineColor.setAlpha(255)
                brushColor = QColor(color)
                brushColor.setAlpha(128)
                painter.setPen(lineColor)
                painter.setBrush(brushColor)
                painter.drawPolygon(self.locations[location])

        if (self.current_selection is not None) or (self.new_selection is not None):
            lineColor = QColor(self.current_selection_color)
            lineColor.setAlpha(255)
            brushColor = QColor(self.current_selection_color)
            brushColor.setAlpha(128)
            painter.setPen(lineColor)
            painter.setBrush(brushColor)
            if self.new_selection is not None:
                # Create a temporary polygon as the new selection is being drawn.
                if self.current_selection is not None:
                    current_selection = QPolygon(self.current_selection)
                    if self.subtract_new_selection:
                        current_selection = current_selection.subtracted(self.new_selection)
                    else:
                        current_selection = current_selection.united(self.new_selection)
                    painter.drawPolygon(current_selection)
                elif self.subtract_new_selection == False:
                    painter.drawPolygon(self.new_selection)
            else:
                painter.drawPolygon(self.current_selection)
        painter.end()

        if rect is None:
            self.image.update()
        else:
            self.image.update(rect)

    def getLocationNameFromPoint(self, point):
        for location in self.locations:
            if self.locations[location].containsPoint(point, Qt.OddEvenFill):
                return location
        return None

    def getUniqueColor(self):
        """
        Use golden ratio to generate unique colors.
        http://martin.ankerl.com/2009/12/09/how-to-create-random-colors-programmatically/
        """
        h = int(359 * (self.new_loc_counter * 0.618033988749895))
        h = h % 359 
        return QColor.fromHsv(h, 255, 255)

    def get_rectangular_polygon(self, pt1, pt2):
        return QPolygon([pt1, QPoint(pt1.x(), pt2.y()), pt2, QPoint(pt2.x(), pt1.y())])

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
        self.function_layout.addStretch(1)

        self.master_layout.addWidget(self.get_horizontal_line())

        # Subfunction toolbar
        self.subfunction_layout = QHBoxLayout()
        clearLayoutAndFixHeight(self.subfunction_layout)
        self.master_layout.addLayout(self.subfunction_layout)
        self.current_subfunction = None

        self.master_layout.addWidget(self.get_horizontal_line())

        self.image = MapImage(self.master_widget)
        self.master_layout.addWidget(self.image)

        self.master_layout.addWidget(self.get_horizontal_line())

        # Configuration toolbar
        self.configuration_layout = QHBoxLayout()
        clearLayoutAndFixHeight(self.configuration_layout)
        self.master_layout.addLayout(self.configuration_layout)

        # Add a stretch at the bottom.
        self.master_layout.addStretch(1)

        self.master_widget.setObjectName('LogicalMarkerPluginUI')
        if context.serial_number() > 1:
            self.master_widget.setWindowTitle(self.master_widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self.master_widget)


        # Activate the functions
        self.functions = {}
        self.functions['Locations'] = LocationFunction(self.master_widget, 
                                                       self.subfunction_layout, 
                                                       self.configuration_layout,
                                                       self.image)

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

        if self.current_function is not None:
            self.functions[self.current_function].deactivateFunction()
        self.current_function = source.text()

        # Clear all subfunction buttons.
        clearLayoutAndFixHeight(self.subfunction_layout)

        if self.current_function is not None:
            self.functions[self.current_function].activateFunction()

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
