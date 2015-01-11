from functools import partial
import os.path
from python_qt_binding.QtCore import QPoint, Qt
from python_qt_binding.QtGui import QFrame, QHBoxLayout, QImage, QLabel, QLineEdit, QMessageBox, \
                                    QPainter, QPolygon, QPushButton, QColor, QVBoxLayout, \
                                    QWidget
from qt_gui.plugin import Plugin
import rospy

import yaml

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
        self.overlay_image = QImage(self.map_image.size(), QImage.Format_ARGB32_Premultiplied) 
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
    EDIT_EXISTING_AREA = 'Edit Location'

    def __init__(self,
                 location_file,
                 widget,
                 subfunction_layout,
                 configuration_layout,
                 image):

        self.edit_area_button = None
        self.edit_area_selection_color = Qt.black

        # Dictionary of polygons
        self.locations = {}
        # Dictionary that maps location names to their colors
        self.location_colors = {}
        self.draw_location = {}
        self.unique_loc_counter = 1

        self.editing_area = False
        self.edit_existing_location = None

        self.editing_properties = False
        self.edit_properties_location = None

        # Use this to initialize variables.
        self.clearAreaSelection()

        self.is_modified = False

        self.widget = widget
        self.subfunction_layout = subfunction_layout
        self.image = image
        self.configuration_layout = configuration_layout

        self.location_file = location_file
        self.readLocationsFromFile()

        self.edit_area_button = {}

    def readLocationsFromFile(self):

        if os.path.isfile(self.location_file):
            stream = open(self.location_file, 'r')
            try:
                contents = yaml.load(stream)
                if "polygons" not in contents or "locations" not in contents:
                    rospy.logerr("YAML file found at " + self.location_file + ", but does not seem to have been written by this tool. I'm starting locations from scratch.")
                else:
                    location_keys = contents["locations"]
                    location_polygons = contents["polygons"]
                    for index, location in enumerate(location_keys):
                        self.locations[location] = QPolygon()
                        self.locations[location].setPoints(location_polygons[index])
                        (_,self.location_colors[location]) = self.getUniqueNameAndColor()
                        self.draw_location[location] = True
            except yaml.YAMLError:
                rospy.logerr("File found at " + self.location_file + ", but cannot be parsed by YAML parser. I'm starting locations from scratch.")

            stream.close()
        else:
            rospy.logwarn("Location file not found at " + self.location_file + ". I'm starting locations from scratch and will attempt to write to this location before exiting.")

    def saveConfiguration(self):
        self.writeLocationsToFile()

    def writeLocationsToFile(self):

        out_dict = {}
        out_dict["locations"] = self.locations.keys()
        out_dict["polygons"] = []
        for index, location in enumerate(self.locations):
            out_dict["polygons"].append([])
            for i in range(self.locations[location].size()):
                pt = self.locations[location].point(i)
                out_dict["polygons"][index].append(pt.x())
                out_dict["polygons"][index].append(pt.y())

        yaml_file_dir = os.path.dirname(os.path.realpath(self.location_file))
        image_file = yaml_file_dir + '/locations.pgm'

        # Create an image with the location data, so that C++ programs don't need to rely on determining regions using polygons.
        out_dict["data"] = 'locations.pgm'
        location_image = QImage(self.image.overlay_image.size(), QImage.Format_RGB32)
        location_image.fill(Qt.white)
        painter = QPainter(location_image) 
        for index, location in enumerate(self.locations):
            if index > 254:
                rospy.logerr("You have more than 255 locations, which is unsupported by the bwi_planning_common C++ code!")
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(index + 1, index + 1, index + 1))
            painter.drawPolygon(self.locations[location])
        painter.end()
        location_image.save(image_file)

        stream = open(self.location_file, 'w')
        yaml.dump(out_dict, stream)
        stream.close()

        self.is_modified = False

    def deactivateFunction(self):

        clearLayoutAndFixHeight(self.subfunction_layout)
        self.edit_area_button.clear()
        self.image.enableDefaultMouseHooks()

        # Just in case we were editing a location, that location was not being drawn. 
        for location in self.draw_location:
            self.draw_location[location] = True

        if self.editing_area:
            self.endAreaEdit("Cancel")
        elif self.editing_properties:
            self.endPropertyEdit()

    def activateFunction(self):

        # Add all the necessary buttons to the subfunction layout.
        clearLayoutAndFixHeight(self.subfunction_layout)
        for button_text in [LocationFunction.ADD_LOCATION, 
                            LocationFunction.EDIT_EXISTING_AREA]:
            button = QPushButton(button_text, self.widget)
            button.clicked[bool].connect(partial(self.startAreaEdit, button_text))
            button.setCheckable(True)
            self.subfunction_layout.addWidget(button)
            self.edit_area_button[button_text] = button
        self.edit_area_button[LocationFunction.EDIT_EXISTING_AREA].setEnabled(False)
        self.subfunction_layout.addStretch(1)

        # ActivateMouseHooks.
        self.image.mousePressEvent = self.mousePressEvent
        self.image.mouseMoveEvent = self.mouseMoveEvent
        self.image.mouseReleaseEvent = self.mouseReleaseEvent

        self.updateOverlay()

    def startAreaEdit(self, edit_type):

        if self.editing_properties:
            self.endPropertyEdit()

        self.editing_area = True

        if edit_type == LocationFunction.ADD_LOCATION:
            self.edit_existing_location = None
        # else edit_existing_location was set to the correct location by startPropertyEdit()

        # Make sure all active selections have been cleared.
        self.clearAreaSelection()

        # If we're going to edit an existing area, stop drawing it and copy it to the active selection.
        if self.edit_existing_location is not None:
            self.draw_location[self.edit_existing_location] = False 
            self.current_selection = QPolygon(self.locations[self.edit_existing_location])
            self.edit_existing_location = self.edit_existing_location

        # Setup the buttons in the configuration toolbar, and disable the original buttons to edit an area.
        clearLayoutAndFixHeight(self.configuration_layout)
        for button_text in ["Done", "Cancel"]:
            button = QPushButton(button_text, self.widget)
            button.clicked[bool].connect(partial(self.endAreaEdit, button_text))
            self.configuration_layout.addWidget(button)
        self.configuration_layout.addStretch(1)

        self.edit_area_button[LocationFunction.ADD_LOCATION].setEnabled(False)
        self.edit_area_button[LocationFunction.EDIT_EXISTING_AREA].setEnabled(False)

        self.updateOverlay()

    def clearAreaSelection(self):

        # Make sure all selections are clear.
        self.new_selection_start_point = None
        self.new_selection_end_point = None

        # QPolygons to track current location.
        self.current_selection = None
        self.new_selection = None
        self.subtract_new_selection = None

    def endAreaEdit(self, button_text):

        edit_properties_location = None

        if (button_text == "Done") and (self.current_selection is not None) and (not self.current_selection.isEmpty()):

            # If the current location being added completely wipes out an old location, make sure you remove it.
            for location in self.locations.keys():
                if location != self.edit_existing_location:
                    self.locations[location] = self.locations[location].subtracted(self.current_selection) 
                    if self.locations[location].isEmpty():
                        self.removeLocation(location)

            if self.edit_existing_location == None:
                # We're adding a new location. Generate a new location name and color.
                (self.edit_existing_location, new_location_color) = self.getUniqueNameAndColor()
                self.location_colors[self.edit_existing_location] = new_location_color
            self.locations[self.edit_existing_location] = self.current_selection
            self.draw_location[self.edit_existing_location] = True
            edit_properties_location = self.edit_existing_location

            # Since a location was added or edited, set file modification to true.
            self.is_modified = True
        else:
            # Cancel was pressed, draw the original location if we were editing as before.
            if self.edit_existing_location is not None:
                self.draw_location[self.edit_existing_location] = True

        self.editing_area = False
        self.edit_existing_location = None
        self.clearAreaSelection()

        # Update the entire image overlay.
        self.updateOverlay()

        self.edit_area_button[LocationFunction.ADD_LOCATION].setEnabled(True)
        self.edit_area_button[LocationFunction.ADD_LOCATION].setChecked(False)
        self.edit_area_button[LocationFunction.EDIT_EXISTING_AREA].setChecked(False)
        clearLayoutAndFixHeight(self.configuration_layout)

        if edit_properties_location is not None:
            self.edit_properties_location = edit_properties_location
            self.startPropertyEdit()

    def startPropertyEdit(self):

        self.editing_properties = True
        self.edit_existing_location = self.edit_properties_location

        self.edit_area_button[LocationFunction.ADD_LOCATION].setEnabled(True)
        self.edit_area_button[LocationFunction.EDIT_EXISTING_AREA].setEnabled(True)

        # Construct the configuration layout.
        clearLayoutAndFixHeight(self.configuration_layout)

        self.update_name_label = QLabel("Location (" + self.edit_properties_location + ")      New Name: ", self.widget)
        self.configuration_layout.addWidget(self.update_name_label)

        self.update_name_textedit = QLineEdit(self.widget)
        self.update_name_textedit.setText(self.edit_properties_location)
        self.update_name_textedit.textEdited.connect(self.locationNameTextEdited)
        self.configuration_layout.addWidget(self.update_name_textedit)

        self.update_name_button = QPushButton("Update location Name", self.widget)
        self.update_name_button.clicked[bool].connect(self.updateLocationName)
        self.update_name_button.setEnabled(False)
        self.configuration_layout.addWidget(self.update_name_button)

        self.remove_location_button = QPushButton("Remove Location", self.widget)
        self.remove_location_button.clicked[bool].connect(self.removeCurrentLocation)
        self.configuration_layout.addWidget(self.remove_location_button)

        self.configuration_layout.addStretch(1)

        self.updateOverlay()

    def endPropertyEdit(self):

        self.edit_area_button[LocationFunction.ADD_LOCATION].setEnabled(True)
        self.edit_area_button[LocationFunction.EDIT_EXISTING_AREA].setEnabled(False)

        clearLayoutAndFixHeight(self.configuration_layout)

        self.update_name_label = None
        self.update_name_textedit = None
        self.update_name_button = None

        self.editing_properties = False

        self.edit_properties_location = None

        self.updateOverlay()

    def locationNameTextEdited(self, text):
        if text != self.edit_properties_location:
            self.update_name_button.setEnabled(True)
        else:
            self.update_name_button.setEnabled(False)

    def updateLocationName(self):
        old_loc_name = self.edit_properties_location
        new_loc_name = self.update_name_textedit.text()

        if new_loc_name in self.locations:
            # This means that two locations need to be merged
            self.locations[new_loc_name] = self.locations[new_loc_name].united(self.locations.pop(old_loc_name))
        else:
            # This is a simple rename task.
            self.locations[new_loc_name] = self.locations.pop(old_loc_name)
            self.location_colors[new_loc_name] = self.location_colors.pop(old_loc_name)
            self.draw_location[new_loc_name] = self.draw_location.pop(old_loc_name)

        # Since a location name was modified, set file modification to true.
        self.is_modified = True

        # Restart property edit with the updated name.
        self.endPropertyEdit()
        self.edit_properties_location = new_loc_name
        self.startPropertyEdit()

    def removeCurrentLocation(self):
        old_loc_name = self.edit_properties_location
        self.removeLocation(old_loc_name)
        self.endPropertyEdit()
        self.updateOverlay()

        # Since a location was removed, set file modification to true.
        self.is_modified = True

    def removeLocation(self, loc_name):
        if loc_name in self.locations:
            self.locations.pop(loc_name)
        if loc_name in self.location_colors:
            self.location_colors.pop(loc_name)
        if loc_name in self.draw_location:
            self.draw_location.pop(loc_name)

    def isModified(self):
        return self.is_modified

    def mousePressEvent(self, event):
        if self.editing_area:
            self.subtract_new_selection = event.button() == Qt.RightButton
            self.new_selection_start_point = event.pos()
            self.new_selection_end_point = event.pos()
            self.new_selection = None
        else:
            loc = self.getLocationNameFromPoint(event.pos()) 
            if loc is not None:
                self.edit_properties_location = loc
                self.startPropertyEdit()
            else:
                self.endPropertyEdit()

    def mouseReleaseEvent(self, event):
        if self.editing_area:
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

        if self.editing_area:

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
                if self.edit_properties_location == location and self.editing_properties:
                    color = self.edit_area_selection_color
                lineColor = QColor(color)
                lineColor.setAlpha(255)
                brushColor = QColor(color)
                brushColor.setAlpha(128)
                painter.setPen(lineColor)
                painter.setBrush(brushColor)
                painter.drawPolygon(self.locations[location])

        if (self.current_selection is not None) or (self.new_selection is not None):
            lineColor = QColor(self.edit_area_selection_color)
            lineColor.setAlpha(255)
            brushColor = QColor(self.edit_area_selection_color)
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

    def getUniqueNameAndColor(self):
        """
        Use golden ratio to generate unique colors.
        http://martin.ankerl.com/2009/12/09/how-to-create-random-colors-programmatically/
        """
        name = "new_loc" + str(self.unique_loc_counter)
        h = int(359 * (self.unique_loc_counter * 0.618033988749895))
        h = h % 359 
        self.unique_loc_counter += 1
        return name, QColor.fromHsv(h, 255, 255)

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
        self.functions['Locations'] = LocationFunction('/home/piyushk/test.yaml',
                                                       self.master_widget, 
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
        modified = False
        for function in self.functions:
            if self.functions[function].isModified():
                modified = True
        if modified:
            ret = QMessageBox.warning(self.master_widget, "Save",
                        "The map has been modified.\n"
                        "Do you want to save your changes?",
                        QMessageBox.Save | QMessageBox.Discard)
            if ret == QMessageBox.Save:
                for function in self.functions:
                    if self.functions[function].isModified():
                        self.functions[function].saveConfiguration()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
