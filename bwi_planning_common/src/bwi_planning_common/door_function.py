#!/bin/env python

from functools import partial
import os.path
from python_qt_binding.QtCore import QPoint, Qt
from python_qt_binding.QtGui import QImage, QLabel, QLineEdit, QPainter, QPolygon, QPushButton, QColor
import rospy
import yaml

from .utils import clearLayoutAndFixHeight

class DoorFunction(object):

    EDIT_DOOR_PROPERITIES = 'Edit Door Properties'
    ADD_NEW_DOOR = 'Add Door'
    EDIT_EXISTING_DOOR = 'Edit Door'

    def __init__(self,
                 door_file,
                 widget,
                 subfunction_layout,
                 configuration_layout,
                 image):

        self.edit_area_button = None
        self.edit_area_selection_color = Qt.black

        # Dictionary of polygons
        self.doors = {}
        # Dictionary that maps door names to their colors
        self.door_colors = {}
        self.draw_door = {}
        self.unique_loc_counter = 1

        self.editing_area = False
        self.edit_existing_door = None

        self.editing_properties = False
        self.edit_properties_door = None

        # Use this to initialize variables.
        self.clearCurrentSelection()

        self.is_modified = False

        self.widget = widget
        self.subfunction_layout = subfunction_layout
        self.image = image
        self.configuration_layout = configuration_layout

        self.door_file = door_file
        self.readDoorsFromFile()

        self.edit_area_button = {}

    def readDoorsFromFile(self):

        if os.path.isfile(self.door_file):
            stream = open(self.door_file, 'r')
            try:
                contents = yaml.load(stream)
                if "polygons" not in contents or "doors" not in contents:
                    rospy.logerr("YAML file found at " + self.door_file + ", but does not seem to have been written by this tool. I'm starting doors from scratch.")
                else:
                    door_keys = contents["doors"]
                    door_polygons = contents["polygons"]
                    for index, door in enumerate(door_keys):
                        self.doors[door] = QPolygon()
                        self.doors[door].setPoints(door_polygons[index])
                        (_,self.door_colors[door]) = self.getUniqueNameAndColor()
                        self.draw_door[door] = True
            except yaml.YAMLError:
                rospy.logerr("File found at " + self.door_file + ", but cannot be parsed by YAML parser. I'm starting doors from scratch.")

            stream.close()
        else:
            rospy.logwarn("Door file not found at " + self.door_file + ". I'm starting doors from scratch and will attempt to write to this door before exiting.")

    def saveConfiguration(self):
        self.writeDoorsToFile()

    def writeDoorsToFile(self):

        out_dict = {}
        out_dict["doors"] = self.doors.keys()
        out_dict["polygons"] = []
        for index, door in enumerate(self.doors):
            out_dict["polygons"].append([])
            for i in range(self.doors[door].size()):
                pt = self.doors[door].point(i)
                out_dict["polygons"][index].append(pt.x())
                out_dict["polygons"][index].append(pt.y())

        yaml_file_dir = os.path.dirname(os.path.realpath(self.door_file))
        image_file = yaml_file_dir + '/doors.pgm'

        # Create an image with the door data, so that C++ programs don't need to rely on determining regions using polygons.
        out_dict["data"] = 'doors.pgm'
        door_image = QImage(self.image.overlay_image.size(), QImage.Format_RGB32)
        door_image.fill(Qt.white)
        painter = QPainter(door_image) 
        for index, door in enumerate(self.doors):
            if index > 254:
                rospy.logerr("You have more than 255 doors, which is unsupported by the bwi_planning_common C++ code!")
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(index + 1, index + 1, index + 1))
            painter.drawPolygon(self.doors[door])
        painter.end()
        door_image.save(image_file)

        stream = open(self.door_file, 'w')
        yaml.dump(out_dict, stream)
        stream.close()

        self.is_modified = False

    def deactivateFunction(self):

        clearLayoutAndFixHeight(self.subfunction_layout)
        self.edit_area_button.clear()
        self.image.enableDefaultMouseHooks()

        # Just in case we were editing a door, that door was not being drawn. 
        for door in self.draw_door:
            self.draw_door[door] = True

        if self.editing_area:
            self.endDoorLocationEdit("Cancel")
        elif self.editing_properties:
            self.endPropertyEdit()

    def activateFunction(self):

        # Add all the necessary buttons to the subfunction layout.
        clearLayoutAndFixHeight(self.subfunction_layout)
        for button_text in [DoorFunction.ADD_NEW_DOOR, 
                            DoorFunction.EDIT_EXISTING_DOOR]:
            button = QPushButton(button_text, self.widget)
            button.clicked[bool].connect(partial(self.startDoorLocationEdit, button_text))
            button.setCheckable(True)
            self.subfunction_layout.addWidget(button)
            self.edit_area_button[button_text] = button
        self.edit_area_button[DoorFunction.EDIT_EXISTING_DOOR].setEnabled(False)
        self.subfunction_layout.addStretch(1)

        # ActivateMouseHooks.
        self.image.mousePressEvent = self.mousePressEvent
        self.image.mouseMoveEvent = self.mouseMoveEvent
        self.image.mouseReleaseEvent = self.mouseReleaseEvent

        self.updateOverlay()

    def getDoorNameFromPoint(self, point):
        for door in self.doors:
            if self.doors[door].containsPoint(point, Qt.OddEvenFill):
                return door
        return None

    def startDoorLocationEdit(self, edit_type):

        if self.editing_properties:
            self.endPropertyEdit()

        self.editing_area = True

        if edit_type == DoorFunction.ADD_NEW_DOOR:
            self.edit_existing_door = None
        # else edit_existing_door was set to the correct door by startPropertyEdit()

        # Make sure all active selections have been cleared.
        self.clearCurrentSelection()

        # If we're going to edit an existing area, stop drawing it and copy it to the active selection.
        if self.edit_existing_door is not None:
            self.draw_door[self.edit_existing_door] = False 
            self.current_selection = QPolygon(self.doors[self.edit_existing_door])
            self.edit_existing_door = self.edit_existing_door

        # Setup the buttons in the configuration toolbar, and disable the original buttons to edit an area.
        clearLayoutAndFixHeight(self.configuration_layout)
        for button_text in ["Done", "Cancel"]:
            button = QPushButton(button_text, self.widget)
            button.clicked[bool].connect(partial(self.endDoorLocationEdit, button_text))
            self.configuration_layout.addWidget(button)
        self.configuration_layout.addStretch(1)

        self.edit_area_button[DoorFunction.ADD_NEW_DOOR].setEnabled(False)
        self.edit_area_button[DoorFunction.EDIT_EXISTING_DOOR].setEnabled(False)

        self.updateOverlay()

    def clearCurrentSelection(self):

        # Make sure all selections are clear.
        self.new_selection_start_point = None
        self.new_selection_end_point = None

        # QPolygons to track current door.
        self.current_selection = None
        self.new_selection = None
        self.subtract_new_selection = None

    def endDoorLocationEdit(self, button_text):

        edit_properties_door = None

        if (button_text == "Done") and (self.current_selection is not None) and (not self.current_selection.isEmpty()):

            # If the current door being added completely wipes out an old door, make sure you remove it.
            for door in self.doors.keys():
                if door != self.edit_existing_door:
                    self.doors[door] = self.doors[door].subtracted(self.current_selection) 
                    if self.doors[door].isEmpty():
                        self.removeDoor(door)

            if self.edit_existing_door == None:
                # We're adding a new door. Generate a new door name and color.
                (self.edit_existing_door, new_door_color) = self.getUniqueNameAndColor()
                self.door_colors[self.edit_existing_door] = new_door_color
            self.doors[self.edit_existing_door] = self.current_selection
            self.draw_door[self.edit_existing_door] = True
            edit_properties_door = self.edit_existing_door

            # Since a door was added or edited, set file modification to true.
            self.is_modified = True
        else:
            # Cancel was pressed, draw the original door if we were editing as before.
            if self.edit_existing_door is not None:
                self.draw_door[self.edit_existing_door] = True

        self.editing_area = False
        self.edit_existing_door = None
        self.clearCurrentSelection()

        # Update the entire image overlay.
        self.updateOverlay()

        self.edit_area_button[DoorFunction.ADD_NEW_DOOR].setEnabled(True)
        self.edit_area_button[DoorFunction.ADD_NEW_DOOR].setChecked(False)
        self.edit_area_button[DoorFunction.EDIT_EXISTING_DOOR].setChecked(False)
        clearLayoutAndFixHeight(self.configuration_layout)

        if edit_properties_door is not None:
            self.edit_properties_door = edit_properties_door
            self.startPropertyEdit()

    def startPropertyEdit(self):

        self.editing_properties = True
        self.edit_existing_door = self.edit_properties_door

        self.edit_area_button[DoorFunction.ADD_NEW_DOOR].setEnabled(True)
        self.edit_area_button[DoorFunction.EDIT_EXISTING_DOOR].setEnabled(True)

        # Construct the configuration layout.
        clearLayoutAndFixHeight(self.configuration_layout)

        self.update_name_label = QLabel("Door (" + self.edit_properties_door + ")      New Name: ", self.widget)
        self.configuration_layout.addWidget(self.update_name_label)

        self.update_name_textedit = QLineEdit(self.widget)
        self.update_name_textedit.setText(self.edit_properties_door)
        self.update_name_textedit.textEdited.connect(self.doorNameTextEdited)
        self.configuration_layout.addWidget(self.update_name_textedit)

        self.update_name_button = QPushButton("Update door Name", self.widget)
        self.update_name_button.clicked[bool].connect(self.updateDoorName)
        self.update_name_button.setEnabled(False)
        self.configuration_layout.addWidget(self.update_name_button)

        self.remove_door_button = QPushButton("Remove Door", self.widget)
        self.remove_door_button.clicked[bool].connect(self.removeCurrentDoor)
        self.configuration_layout.addWidget(self.remove_door_button)

        self.configuration_layout.addStretch(1)

        self.updateOverlay()

    def endPropertyEdit(self):

        self.edit_area_button[DoorFunction.ADD_NEW_DOOR].setEnabled(True)
        self.edit_area_button[DoorFunction.EDIT_EXISTING_DOOR].setEnabled(False)

        clearLayoutAndFixHeight(self.configuration_layout)

        self.update_name_label = None
        self.update_name_textedit = None
        self.update_name_button = None

        self.editing_properties = False

        self.edit_properties_door = None

        self.updateOverlay()

    def doorNameTextEdited(self, text):
        if text != self.edit_properties_door:
            self.update_name_button.setEnabled(True)
        else:
            self.update_name_button.setEnabled(False)

    def updateDoorName(self):
        old_loc_name = self.edit_properties_door
        new_loc_name = self.update_name_textedit.text()

        if new_loc_name in self.doors:
            # This means that two doors need to be merged
            self.doors[new_loc_name] = self.doors[new_loc_name].united(self.doors.pop(old_loc_name))
        else:
            # This is a simple rename task.
            self.doors[new_loc_name] = self.doors.pop(old_loc_name)
            self.door_colors[new_loc_name] = self.door_colors.pop(old_loc_name)
            self.draw_door[new_loc_name] = self.draw_door.pop(old_loc_name)

        # Since a door name was modified, set file modification to true.
        self.is_modified = True

        # Restart property edit with the updated name.
        self.endPropertyEdit()
        self.edit_properties_door = new_loc_name
        self.startPropertyEdit()

    def removeCurrentDoor(self):
        old_loc_name = self.edit_properties_door
        self.removeDoor(old_loc_name)
        self.endPropertyEdit()
        self.updateOverlay()

        # Since a door was removed, set file modification to true.
        self.is_modified = True

    def removeDoor(self, loc_name):
        if loc_name in self.doors:
            self.doors.pop(loc_name)
        if loc_name in self.door_colors:
            self.door_colors.pop(loc_name)
        if loc_name in self.draw_door:
            self.draw_door.pop(loc_name)

    def isModified(self):
        return self.is_modified

    def mousePressEvent(self, event):
        if self.editing_area:
            self.subtract_new_selection = event.button() == Qt.RightButton
            self.new_selection_start_point = event.pos()
            self.new_selection_end_point = event.pos()
            self.new_selection = None
        else:
            loc = self.getDoorNameFromPoint(event.pos()) 
            if loc is not None:
                self.edit_properties_door = loc
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
        # Redraw the overlay image from scratch using the door image and current door.

        self.image.overlay_image.fill(Qt.transparent)
        painter = QPainter(self.image.overlay_image)
        painter.setBackgroundMode(Qt.TransparentMode)
        painter.setCompositionMode(QPainter.CompositionMode_Source)

        for door in self.doors:
            if self.draw_door[door]:
                color = self.door_colors[door]
                if self.edit_properties_door == door and self.editing_properties:
                    color = self.edit_area_selection_color
                lineColor = QColor(color)
                lineColor.setAlpha(255)
                brushColor = QColor(color)
                brushColor.setAlpha(128)
                painter.setPen(lineColor)
                painter.setBrush(brushColor)
                painter.drawPolygon(self.doors[door])

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

