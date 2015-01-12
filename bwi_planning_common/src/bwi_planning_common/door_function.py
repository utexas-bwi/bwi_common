#!/bin/env python

from functools import partial
import math
import os.path
from python_qt_binding.QtCore import QPoint, Qt
from python_qt_binding.QtGui import QLabel, QLineEdit, QPainter, QPolygon, QPushButton
import rospy
import yaml

from .utils import clearLayoutAndFixHeight

class Door(object):

    def __init__(self,
                 door_corner_pt_1,
                 door_corner_pt_2,
                 approach_pt_1,
                 approach_pt_2):

        self.door_corner_pt_1 = door_corner_pt_1
        self.door_corner_pt_2 = door_corner_pt_2
        self.approach_pt_1 = approach_pt_1
        self.approach_pt_2 = approach_pt_2

    def clone(self):
        return Door(QPoint(self.door_corner_pt_1),
                    QPoint(self.door_corner_pt_2),
                    QPoint(self.approach_pt_1),
                    QPoint(self.approach_pt_2))

class DoorFunction(object):

    EDIT_DOOR_PROPERITIES = 'Edit Door Properties'
    ADD_NEW_DOOR = 'Add Door'
    EDIT_EXISTING_DOOR = 'Edit Door'

    def __init__(self,
                 door_file,
                 map,
                 location_function,
                 widget,
                 subfunction_layout,
                 configuration_layout,
                 image):

        self.edit_door_location_button = None
        self.selected_door_color = Qt.blue
        self.unselected_door_color = Qt.darkGreen

        # Dictionary that maps door names to the actual door object (string->Door)
        self.doors = {}
        self.draw_door = {}
        self.unique_loc_counter = 1

        self.editing_door_location = False
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
        self.map = map
        self.location_function = location_function
        self.readDoorsFromFile()

        self.edit_door_location_button = {}

    def readDoorsFromFile(self):

        if os.path.isfile(self.door_file):
            stream = open(self.door_file, 'r')
            try:
                contents = yaml.load(stream)
                for door in contents:
                    door_key = door["name"]
                    door_corner_pt_1 = QPoint(*door["door_corner_pt_1"])
                    door_corner_pt_2 = QPoint(*door["door_corner_pt_2"])
                    approach_pts = door["approach"]
                    if len(approach_pts) != 2:
                        rospy.logerr("Door " + door_key + " read from file " + self.door_file + " has " + str(len(approach_pts)) + " approach points instead of 2. Ignoring this door.")
                        continue
                    approach_pt_1 = QPoint(approach_pts[0]["point"][0], approach_pts[0]["point"][1])
                    approach_pt_2 = QPoint(approach_pts[1]["point"][0], approach_pts[1]["point"][1])
                    self.doors[door_key] = Door(door_corner_pt_1,
                                                door_corner_pt_2,
                                                approach_pt_1,
                                                approach_pt_2)
                    self.draw_door[door_key] = True
            except yaml.YAMLError, KeyError:
                rospy.logerr("File found at " + self.door_file + ", but cannot be parsed by YAML parser. I'm starting doors from scratch.")

            stream.close()
        else:
            rospy.logwarn("Door file not found at " + self.door_file + ". I'm starting doors from scratch and will attempt to write to this door before exiting.")

    def saveConfiguration(self):
        self.writeDoorsToFile()

    def writeDoorsToFile(self):

        out_list = []
        for door_name in self.doors:
            door = self.doors[door_name]
            door_dict = {}
            door_dict["name"] = door_name
            door_dict["door_corner_pt_1"] = [door.door_corner_pt_1.x(), door.door_corner_pt_1.y()]
            door_dict["door_corner_pt_2"] = [door.door_corner_pt_2.x(), door.door_corner_pt_2.y()]
            door_dict["approach"] = []
            mid_point = (door.door_corner_pt_1 + door.door_corner_pt_2) / 2
            for point in [door.approach_pt_1, door.approach_pt_2]:
                approach_pt_dict = {}
                approach_pt_dict["from"] = self.location_function.getLocationNameFromPoint(point)
                diff_pt = point - mid_point
                approach_angle = math.atan2(diff_pt.y(), diff_pt.x())
                approach_pt_dict["point"] = [point.x(), point.y(), approach_angle]
                door_dict["approach"].append(approach_pt_dict)
            out_list.append(door_dict)

        stream = open(self.door_file, 'w')
        yaml.dump(out_list, stream)
        stream.close()

        self.is_modified = False

    def deactivateFunction(self):

        if self.editing_door_location:
            self.endDoorLocationEdit("Cancel")
        elif self.editing_properties:
            self.endPropertyEdit()

        clearLayoutAndFixHeight(self.subfunction_layout)
        self.edit_door_location_button.clear()
        self.image.enableDefaultMouseHooks()

        # Just in case we were editing a door, that door was not being drawn. 
        for door in self.draw_door:
            self.draw_door[door] = True

    def activateFunction(self):

        # Add all the necessary buttons to the subfunction layout.
        clearLayoutAndFixHeight(self.subfunction_layout)
        for button_text in [DoorFunction.ADD_NEW_DOOR, 
                            DoorFunction.EDIT_EXISTING_DOOR]:
            button = QPushButton(button_text, self.widget)
            button.clicked[bool].connect(partial(self.startDoorLocationEdit, button_text))
            button.setCheckable(True)
            self.subfunction_layout.addWidget(button)
            self.edit_door_location_button[button_text] = button
        self.edit_door_location_button[DoorFunction.EDIT_EXISTING_DOOR].setEnabled(False)
        self.subfunction_layout.addStretch(1)

        # ActivateMouseHooks.
        self.image.mousePressEvent = self.mousePressEvent
        self.image.mouseMoveEvent = self.mouseMoveEvent
        self.image.mouseReleaseEvent = self.mouseReleaseEvent

        self.updateOverlay()

    def getDoorNameFromPoint(self, point):
        for door in self.doors:
            # Check if the user is clicking on the line between the doors or the two approach points.
            if self.getPointDistanceToAnotherPoint(point, self.doors[door].approach_pt_1) <= 3:
                return door
            if self.getPointDistanceToAnotherPoint(point, self.doors[door].approach_pt_2) <= 3:
                return door
            if self.getPointDistanceToLineSegment(point,
                                                  self.doors[door].door_corner_pt_1,
                                                  self.doors[door].door_corner_pt_2) <= 3:
                return door
        return None

    def startDoorLocationEdit(self, edit_type):

        if self.editing_properties:
            self.endPropertyEdit()

        self.editing_door_location = True

        if edit_type == DoorFunction.ADD_NEW_DOOR:
            self.edit_existing_door = None
        # else edit_existing_door was set to the correct door by startPropertyEdit()

        # Make sure all active selections have been cleared.
        self.clearCurrentSelection()

        # If we're going to edit an existing area, stop drawing it and copy it to the active selection.
        if self.edit_existing_door is not None:
            self.draw_door[self.edit_existing_door] = False 
            self.current_selection = self.doors[self.edit_existing_door].clone()
            self.edit_existing_door = self.edit_existing_door

        # Setup the buttons in the configuration toolbar, and disable the original buttons to edit an area.
        clearLayoutAndFixHeight(self.configuration_layout)
        for button_text in ["Done", "Cancel"]:
            button = QPushButton(button_text, self.widget)
            button.clicked[bool].connect(partial(self.endDoorLocationEdit, button_text))
            self.configuration_layout.addWidget(button)
        self.current_selection_label = QLabel(self.widget)
        self.configuration_layout.addWidget(self.current_selection_label)
        self.configuration_layout.addStretch(1)

        self.edit_door_location_button[DoorFunction.ADD_NEW_DOOR].setEnabled(False)
        self.edit_door_location_button[DoorFunction.EDIT_EXISTING_DOOR].setEnabled(False)

        self.updateOverlay()

    def clearCurrentSelection(self):

        # Make sure all selections are clear.
        self.new_selection_start_point = None
        self.new_selection_end_point = None

        # Door
        self.current_selection = None
        self.current_selection_label = None # Displays which two locations the door is connecting.
        self.move_selection = None

    def endDoorLocationEdit(self, button_text):

        edit_properties_door = None

        if (button_text == "Done") and (self.current_selection is not None):

            if self.edit_existing_door == None:
                # We're adding a new door. Generate a new door name and color.
                self.edit_existing_door = self.getUniqueName()
            self.doors[self.edit_existing_door] = self.current_selection
            self.draw_door[self.edit_existing_door] = True
            edit_properties_door = self.edit_existing_door

            # Since a door was added or edited, set file modification to true.
            self.is_modified = True
        else:
            # Cancel was pressed, draw the original door if we were editing as before.
            if self.edit_existing_door is not None:
                self.draw_door[self.edit_existing_door] = True

        self.editing_door_location = False
        self.edit_existing_door = None
        self.clearCurrentSelection()

        # Update the entire image overlay.
        self.updateOverlay()

        self.edit_door_location_button[DoorFunction.ADD_NEW_DOOR].setEnabled(True)
        self.edit_door_location_button[DoorFunction.ADD_NEW_DOOR].setChecked(False)
        self.edit_door_location_button[DoorFunction.EDIT_EXISTING_DOOR].setChecked(False)
        clearLayoutAndFixHeight(self.configuration_layout)

        if edit_properties_door is not None:
            self.edit_properties_door = edit_properties_door
            self.startPropertyEdit()

    def startPropertyEdit(self):

        self.editing_properties = True
        self.edit_existing_door = self.edit_properties_door

        self.edit_door_location_button[DoorFunction.ADD_NEW_DOOR].setEnabled(True)
        self.edit_door_location_button[DoorFunction.EDIT_EXISTING_DOOR].setEnabled(True)

        # Construct the configuration layout.
        clearLayoutAndFixHeight(self.configuration_layout)

        connects_text = self.selfGetConnectingText(self.doors[self.edit_properties_door])
        self.update_name_label = QLabel("Door (" + self.edit_properties_door + " - " + connects_text + ")      New Name: ", self.widget)
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

        self.edit_door_location_button[DoorFunction.ADD_NEW_DOOR].setEnabled(True)
        self.edit_door_location_button[DoorFunction.EDIT_EXISTING_DOOR].setEnabled(False)

        clearLayoutAndFixHeight(self.configuration_layout)

        self.update_name_label = None
        self.update_name_textedit = None
        self.update_name_button = None

        self.editing_properties = False

        self.edit_properties_door = None

        self.updateOverlay()

    def doorNameTextEdited(self, text):
        if str(text) != self.edit_properties_door and str(text) not in self.doors:
            self.update_name_button.setEnabled(True)
        else:
            self.update_name_button.setEnabled(False)

    def updateDoorName(self):
        old_loc_name = self.edit_properties_door
        new_loc_name = str(self.update_name_textedit.text())

        # This is a simple rename task.
        self.doors[new_loc_name] = self.doors.pop(old_loc_name)
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
        if loc_name in self.draw_door:
            self.draw_door.pop(loc_name)

    def isModified(self):
        return self.is_modified

    def mousePressEvent(self, event):
        if self.editing_door_location:
            if self.current_selection == None:
                # We are drawing the original door, and not moving it.
                self.new_selection_start_point = event.pos()
                self.new_selection_end_point = event.pos()
                self.move_selection = None
            else:
                # The initial door is already drawn. See if the user wants to move a point.
                self.new_selection_start_point = None
                self.new_selection_end_point = None

                # Check if the user is clicking on 1 of the 4 points that define the door.
                self.move_selection = None
                for pt in [self.current_selection.door_corner_pt_1,
                           self.current_selection.door_corner_pt_2,
                           self.current_selection.approach_pt_1,
                           self.current_selection.approach_pt_2]:
                    if self.getPointDistanceToAnotherPoint(event.pos(), pt) <= 3:
                        self.move_selection = pt
                        break
        else:
            loc = self.getDoorNameFromPoint(event.pos()) 
            if loc is not None:
                self.edit_properties_door = loc
                self.startPropertyEdit()
            else:
                self.endPropertyEdit()

    def mouseReleaseEvent(self, event):
        if self.editing_door_location:
            self.mouseMoveEvent(event)
            if self.current_selection is None:
                midpoint = (self.new_selection_start_point + self.new_selection_end_point) / 2 
                diff = self.new_selection_end_point - self.new_selection_start_point
                segment_angle = math.atan2(diff.y(), diff.x())
                perpendicular_angle = segment_angle + math.pi / 2.0
                perpendicular_diff_pt = QPoint(int(10.0 * math.cos(perpendicular_angle)), 
                                               int(10.0 * math.sin(perpendicular_angle))) 

                approach_pt_1 = midpoint + perpendicular_diff_pt
                approach_pt_2 = midpoint - perpendicular_diff_pt
                self.current_selection = Door(self.new_selection_start_point,
                                              self.new_selection_end_point,
                                              approach_pt_1,
                                              approach_pt_2)
                self.new_selection_start_pt = None
                self.new_selection_end_point = None
                self.updateOverlay()

    def mouseMoveEvent(self, event):

        if self.editing_door_location:
            overlay_update_region = None
            if self.current_selection is None:
                # First make sure we update the region corresponding to the old mark.
                old_overlay_update_rect = self.getRectangularPolygon(self.new_selection_start_point, self.new_selection_end_point)
                self.new_selection_end_point = event.pos()
                # Next determine the region that needs to be update because of the new mark.
                new_overlay_update_rect = self.getRectangularPolygon(self.new_selection_start_point, self.new_selection_end_point)
                overlay_update_region = (old_overlay_update_rect + new_overlay_update_rect).boundingRect()
                overlay_update_region.setTopLeft(QPoint(overlay_update_region.topLeft().x() - 4,
                                                        overlay_update_region.topLeft().y() - 4))
                overlay_update_region.setBottomRight(QPoint(overlay_update_region.bottomRight().x() + 4,
                                                            overlay_update_region.bottomRight().y() + 4))

                self.updateOverlay(overlay_update_region)
            elif self.move_selection is not None:
                # Copy the old point.
                old_loc = QPoint(self.move_selection)
                # Move the current point.
                self.move_selection.setX(event.pos().x())
                self.move_selection.setY(event.pos().y())
                # TODO do some math to figure out the overlay update region.
                self.updateOverlay()

    def updateOverlay(self, rect = None):

        # Redraw the overlay image from scratch using the door image and current door.

        self.image.overlay_image.fill(Qt.transparent)
        painter = QPainter(self.image.overlay_image)
        painter.setBackgroundMode(Qt.TransparentMode)
        painter.setCompositionMode(QPainter.CompositionMode_Source)

        for door in self.doors:
            if self.draw_door[door]:
                color = self.unselected_door_color
                if self.edit_properties_door == door and self.editing_properties:
                    color = self.selected_door_color
                self.drawDoor(self.doors[door], painter, color)

        if self.current_selection is not None:
            color = self.selected_door_color
            self.drawDoor(self.current_selection, painter, color)
        elif self.new_selection_start_point is not None:
            color = self.selected_door_color
            self.drawLine(self.new_selection_start_point, self.new_selection_end_point, painter, color)
        painter.end()

        if rect is None:
            self.image.update()
        else:
            self.image.update(rect)

    def selfGetConnectingText(self, door):
        # Get the two locations this door connects by looking up the locations of the approach points in the 
        # door function.
        approach_location_1 = self.location_function.getLocationNameFromPoint(door.approach_pt_1)
        if approach_location_1 is None:
            approach_location_1 = "<None>"
        approach_location_2 = self.location_function.getLocationNameFromPoint(door.approach_pt_2)
        if approach_location_2 is None:
            approach_location_2 = "<None>"
        return "Connects: " + approach_location_1 + " <-> " + approach_location_2

    def getUniqueName(self):
        name = "new_door" + str(self.unique_loc_counter)
        self.unique_loc_counter += 1
        return name

    def getPointDistanceToAnotherPoint(self, pt1, pt2):
        diff = pt1 - pt2
        return math.sqrt(diff.x() * diff.x() + diff.y() * diff.y())

    def getPointDistanceToLineSegment(self, pt, segment_pt_1, segment_pt_2):
        """
        http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
        """
        px = segment_pt_2.x()-segment_pt_1.x()
        py = segment_pt_2.y()-segment_pt_1.y()

        something = px*px + py*py

        u =  ((pt.x() - segment_pt_1.x()) * px + (pt.y() - segment_pt_1.y()) * py) / float(something)

        if u > 1:
            u = 1
        elif u < 0:
            u = 0

        x = segment_pt_1.x() + u * px
        y = segment_pt_1.y() + u * py

        dx = x - pt.x()
        dy = y - pt.y()

        return math.sqrt(dx*dx + dy*dy)

    def getRectangularPolygon(self, pt1, pt2):
        return QPolygon([pt1, QPoint(pt1.x(), pt2.y()), pt2, QPoint(pt2.x(), pt1.y())])

    def drawDoor(self, door, painter, color):
        self.drawLine(door.door_corner_pt_1, door.door_corner_pt_2, painter, color)
        self.drawPoint(door.approach_pt_1, painter, color)
        self.drawPoint(door.approach_pt_2, painter, color)

    def drawLine(self, pt1, pt2, painter, color):
        painter.setPen(color)
        painter.drawLine(pt1, pt2)
        self.drawPoint(pt1, painter, color)
        self.drawPoint(pt2, painter, color)

    def drawPoint(self, pt, painter, color):
        painter.setPen(color)
        painter.drawPoint(pt)
        painter.drawEllipse(pt, 3, 3)
