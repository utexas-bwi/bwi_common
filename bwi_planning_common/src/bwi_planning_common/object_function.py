#!/bin/env python

from functools import partial
import math
import os.path
import rospy
import yaml

from python_qt_binding.QtCore import QPoint, QPointF, QRect, Qt
from python_qt_binding.QtGui import QPainter, QPolygon
try:
    from python_qt_binding.QtGui import QLabel, QLineEdit, QPushButton
except ImportError:
    from python_qt_binding.QtWidgets import QLabel, QLineEdit, QPushButton

from .utils import clearLayoutAndFixHeight, \
                   transformPointToPixelCoordinates, \
                   transformPointToRealWorldCoordinates

class Object(object):

    def __init__(self,
                 location,
                 orientation):

        self.location = location
        self.orientation = orientation

    def clone(self):
        return Object(QPoint(self.location), self.orientation)

class ObjectFunction(object):

    EDIT_OBJECT_PROPERITIES = 'Edit Object Properties'
    ADD_NEW_OBJECT = 'Add Object'
    EDIT_EXISTING_OBJECT = 'Edit Object'

    ORIENTATION_LENGTH = 10

    def __init__(self,
                 object_file,
                 map,
                 location_function,
                 widget,
                 subfunction_layout,
                 configuration_layout,
                 image):

        self.edit_object_location_button = None
        self.selected_object_color = Qt.blue
        self.unselected_object_color = Qt.darkGreen

        # Dictionary that maps object names to the actual object object (string->Object)
        self.objects = {}
        self.draw_object = {}
        self.unique_loc_counter = 1

        self.editing_object_location = False
        self.edit_existing_object = None

        self.editing_properties = False
        self.edit_properties_object = None

        # Use this to initialize variables.
        self.clearCurrentSelection()

        self.is_modified = False

        self.widget = widget
        self.subfunction_layout = subfunction_layout
        self.image = image
        self.image_size = image.overlay_image.size()
        self.configuration_layout = configuration_layout

        self.object_file = object_file
        self.map = map
        self.location_function = location_function
        self.readObjectsFromFile()

        self.edit_object_location_button = {}

    def readObjectsFromFile(self):

        if os.path.isfile(self.object_file):
            stream = open(self.object_file, 'r')
            try:
                contents = yaml.load(stream)
                for object in contents:
                    object_key = object["name"]
                    object_location = QPointF(object["point"][0], object["point"][1])
                    object_location = transformPointToPixelCoordinates(object_location, self.map, self.image_size)
                    object_orientation = float(object["point"][2])
                    self.objects[object_key] = Object(object_location, object_orientation)
                    self.draw_object[object_key] = True
            except yaml.YAMLError, KeyError:
                rospy.logerr("File found at " + self.object_file + ", but cannot be parsed by YAML parser. I'm starting objects from scratch.")

            stream.close()
        else:
            rospy.logwarn("Object file not found at " + self.object_file + ". I'm starting objects from scratch and will attempt to write to this object before exiting.")

    def saveConfiguration(self):
        self.writeObjectsToFile()

    def writeObjectsToFile(self):

        out_list = []
        for object_name in self.objects:
            object = self.objects[object_name]
            object_location = transformPointToRealWorldCoordinates(object.location, self.map, self.image_size)
            object_dict = {}
            object_dict["name"] = object_name
            object_dict["point"] = [object_location.x(), object_location.y(), object.orientation]
            out_list.append(object_dict)

        stream = open(self.object_file, 'w')
        yaml.dump(out_list, stream)
        stream.close()

        self.is_modified = False

    def deactivateFunction(self):

        if self.editing_object_location:
            self.endObjectLocationEdit("Cancel")
        elif self.editing_properties:
            self.endPropertyEdit()

        clearLayoutAndFixHeight(self.subfunction_layout)
        self.edit_object_location_button.clear()
        self.image.enableDefaultMouseHooks()

        # Just in case we were editing a object, that object was not being drawn. 
        for object in self.draw_object:
            self.draw_object[object] = True

    def activateFunction(self):

        # Add all the necessary buttons to the subfunction layout.
        clearLayoutAndFixHeight(self.subfunction_layout)
        for button_text in [ObjectFunction.ADD_NEW_OBJECT, 
                            ObjectFunction.EDIT_EXISTING_OBJECT]:
            button = QPushButton(button_text, self.widget)
            button.clicked[bool].connect(partial(self.startObjectLocationEdit, button_text))
            button.setCheckable(True)
            self.subfunction_layout.addWidget(button)
            self.edit_object_location_button[button_text] = button
        self.edit_object_location_button[ObjectFunction.EDIT_EXISTING_OBJECT].setEnabled(False)
        self.subfunction_layout.addStretch(1)

        # ActivateMouseHooks.
        self.image.mousePressEvent = self.mousePressEvent
        self.image.mouseMoveEvent = self.mouseMoveEvent
        self.image.mouseReleaseEvent = self.mouseReleaseEvent

        self.updateOverlay()

    def getObjectNameFromPoint(self, point):
        for object in self.objects:
            # Check if the user is clicking on the line between the objects or the two approach points.
            if self.getPointDistanceToAnotherPoint(point, self.objects[object].location) <= 3:
                return object
        return None

    def startObjectLocationEdit(self, edit_type):

        if self.editing_properties:
            self.endPropertyEdit()

        self.editing_object_location = True

        if edit_type == ObjectFunction.ADD_NEW_OBJECT:
            self.edit_existing_object = None
        # else edit_existing_object was set to the correct object by startPropertyEdit()

        # Make sure all active selections have been cleared.
        self.clearCurrentSelection()

        # If we're going to edit an existing area, stop drawing it and copy it to the active selection.
        if self.edit_existing_object is not None:
            self.draw_object[self.edit_existing_object] = False 
            self.current_selection = self.objects[self.edit_existing_object].clone()
            self.edit_existing_object = self.edit_existing_object

        # Setup the buttons in the configuration toolbar, and disable the original buttons to edit an area.
        clearLayoutAndFixHeight(self.configuration_layout)
        for button_text in ["Done", "Cancel"]:
            button = QPushButton(button_text, self.widget)
            button.clicked[bool].connect(partial(self.endObjectLocationEdit, button_text))
            self.configuration_layout.addWidget(button)
        self.current_selection_label = QLabel(self.widget)
        self.configuration_layout.addWidget(self.current_selection_label)
        self.configuration_layout.addStretch(1)

        self.edit_object_location_button[ObjectFunction.ADD_NEW_OBJECT].setEnabled(False)
        self.edit_object_location_button[ObjectFunction.EDIT_EXISTING_OBJECT].setEnabled(False)

        self.updateOverlay()

    def clearCurrentSelection(self):

        # Make sure all selections are clear.
        self.new_selection_start_point = None
        self.new_selection_end_point = None

        # Object
        self.current_selection = None
        self.current_selection_label = None # Displays which two locations the object is connecting.
        self.move_selection = None

    def endObjectLocationEdit(self, button_text):

        edit_properties_object = None

        if (button_text == "Done") and (self.current_selection is not None):

            if self.edit_existing_object == None:
                # We're adding a new object. Generate a new object name and color.
                self.edit_existing_object = self.getUniqueName()
            self.objects[self.edit_existing_object] = self.current_selection
            self.draw_object[self.edit_existing_object] = True
            edit_properties_object = self.edit_existing_object

            # Since a object was added or edited, set file modification to true.
            self.is_modified = True
        else:
            # Cancel was pressed, draw the original object if we were editing as before.
            if self.edit_existing_object is not None:
                self.draw_object[self.edit_existing_object] = True

        self.editing_object_location = False
        self.edit_existing_object = None
        self.clearCurrentSelection()

        # Update the entire image overlay.
        self.updateOverlay()

        self.edit_object_location_button[ObjectFunction.ADD_NEW_OBJECT].setEnabled(True)
        self.edit_object_location_button[ObjectFunction.ADD_NEW_OBJECT].setChecked(False)
        self.edit_object_location_button[ObjectFunction.EDIT_EXISTING_OBJECT].setChecked(False)
        clearLayoutAndFixHeight(self.configuration_layout)

        if edit_properties_object is not None:
            self.edit_properties_object = edit_properties_object
            self.startPropertyEdit()

    def startPropertyEdit(self):

        self.editing_properties = True
        self.edit_existing_object = self.edit_properties_object

        self.edit_object_location_button[ObjectFunction.ADD_NEW_OBJECT].setEnabled(True)
        self.edit_object_location_button[ObjectFunction.EDIT_EXISTING_OBJECT].setEnabled(True)

        # Construct the configuration layout.
        clearLayoutAndFixHeight(self.configuration_layout)

        location_text = self.getObjectLocationText(self.objects[self.edit_properties_object])
        self.update_name_label = QLabel("Object (" + self.edit_properties_object + " - " + location_text + ")      New Name: ", self.widget)
        self.configuration_layout.addWidget(self.update_name_label)

        self.update_name_textedit = QLineEdit(self.widget)
        self.update_name_textedit.setText(self.edit_properties_object)
        self.update_name_textedit.textEdited.connect(self.objectNameTextEdited)
        self.configuration_layout.addWidget(self.update_name_textedit)

        self.update_name_button = QPushButton("Update object Name", self.widget)
        self.update_name_button.clicked[bool].connect(self.updateObjectName)
        self.update_name_button.setEnabled(False)
        self.configuration_layout.addWidget(self.update_name_button)

        self.remove_object_button = QPushButton("Remove Object", self.widget)
        self.remove_object_button.clicked[bool].connect(self.removeCurrentObject)
        self.configuration_layout.addWidget(self.remove_object_button)

        self.configuration_layout.addStretch(1)

        self.updateOverlay()

    def endPropertyEdit(self):

        self.edit_object_location_button[ObjectFunction.ADD_NEW_OBJECT].setEnabled(True)
        self.edit_object_location_button[ObjectFunction.EDIT_EXISTING_OBJECT].setEnabled(False)

        clearLayoutAndFixHeight(self.configuration_layout)

        self.update_name_label = None
        self.update_name_textedit = None
        self.update_name_button = None

        self.editing_properties = False

        self.edit_properties_object = None

        self.updateOverlay()

    def objectNameTextEdited(self, text):
        if str(text) != self.edit_properties_object and str(text) not in self.objects:
            self.update_name_button.setEnabled(True)
        else:
            self.update_name_button.setEnabled(False)

    def updateObjectName(self):
        old_obj_name = self.edit_properties_object
        new_obj_name = str(self.update_name_textedit.text())

        # This is a simple rename task.
        self.objects[new_obj_name] = self.objects.pop(old_obj_name)
        self.draw_object[new_obj_name] = self.draw_object.pop(old_obj_name)

        # Since a object name was modified, set file modification to true.
        self.is_modified = True

        # Restart property edit with the updated name.
        self.endPropertyEdit()
        self.edit_properties_object = new_obj_name
        self.startPropertyEdit()

    def removeCurrentObject(self):
        old_obj_name = self.edit_properties_object
        self.removeObject(old_obj_name)
        self.endPropertyEdit()
        self.updateOverlay()

        # Since a object was removed, set file modification to true.
        self.is_modified = True

    def removeObject(self, obj_name):
        if obj_name in self.objects:
            self.objects.pop(obj_name)
        if obj_name in self.draw_object:
            self.draw_object.pop(obj_name)

    def isModified(self):
        return self.is_modified

    def mousePressEvent(self, event):
        if self.editing_object_location:
            old_selection_location = None
            if self.current_selection is not None:
                # First make sure we copy the region corresponding to the old point.
                old_selection_location = QPoint(self.current_selection.location)
            self.current_selection = Object(event.pos(), 0)
            if old_selection_location is None:
                overlay_update_region = self.getRectangularPolygon(self.current_selection.location,
                                                                   self.current_selection.location).boundingRect()
            else:
                overlay_update_region = self.getRectangularPolygon(old_selection_location,
                                                                   self.current_selection.location).boundingRect()

            overlay_update_region.setTopLeft(QPoint(overlay_update_region.topLeft().x() - (ObjectFunction.ORIENTATION_LENGTH + 1),
                                                    overlay_update_region.topLeft().y() - (ObjectFunction.ORIENTATION_LENGTH + 1)))
            overlay_update_region.setBottomRight(QPoint(overlay_update_region.bottomRight().x() + (ObjectFunction.ORIENTATION_LENGTH + 1),
                                                        overlay_update_region.bottomRight().y() + (ObjectFunction.ORIENTATION_LENGTH + 1)))

            self.current_selection_label.setText(self.getObjectLocationText(self.current_selection))
            self.updateOverlay(overlay_update_region)
        else:
            loc = self.getObjectNameFromPoint(event.pos()) 
            if loc is not None:
                self.edit_properties_object = loc
                self.startPropertyEdit()
            else:
                self.endPropertyEdit()

    def mouseReleaseEvent(self, event):
        if self.editing_object_location:
            self.mouseMoveEvent(event)

    def mouseMoveEvent(self, event):

        if self.editing_object_location:
            diff = event.pos() - self.current_selection.location
            self.current_selection.orientation = math.atan2(diff.y(), diff.x())
            overlay_update_region = QRect(QPoint(self.current_selection.location.x() - (ObjectFunction.ORIENTATION_LENGTH + 1),
                                                 self.current_selection.location.y() - (ObjectFunction.ORIENTATION_LENGTH + 1)),
                                          QPoint(self.current_selection.location.x() + (ObjectFunction.ORIENTATION_LENGTH + 1),
                                                 self.current_selection.location.y() + (ObjectFunction.ORIENTATION_LENGTH + 1)))
            self.updateOverlay(overlay_update_region)

    def updateOverlay(self, rect = None):

        # Redraw the overlay image from scratch using the object image and current object.

        self.image.overlay_image.fill(Qt.transparent)
        painter = QPainter(self.image.overlay_image)
        painter.setBackgroundMode(Qt.TransparentMode)
        painter.setCompositionMode(QPainter.CompositionMode_Source)

        for object in self.objects:
            if self.draw_object[object]:
                color = self.unselected_object_color
                if self.edit_properties_object == object and self.editing_properties:
                    color = self.selected_object_color
                self.drawObject(self.objects[object], painter, color)

        if self.current_selection is not None:
            color = self.selected_object_color
            self.drawObject(self.current_selection, painter, color)
        painter.end()

        if rect is None:
            self.image.update()
        else:
            self.image.update(rect)

    def getObjectLocationText(self, object):
        # Get the two locations this object connects by looking up the locations of the approach points in the 
        # object function.
        location = self.location_function.getLocationNameFromPoint(object.location)
        if location is None:
            location = "<None>"
        return "Lies in: " + location 

    def getUniqueName(self):
        name = "new_object" + str(self.unique_loc_counter)
        self.unique_loc_counter += 1
        return name

    def getPointDistanceToAnotherPoint(self, pt1, pt2):
        diff = pt1 - pt2
        return math.sqrt(diff.x() * diff.x() + diff.y() * diff.y())

    def getRectangularPolygon(self, pt1, pt2):
        return QPolygon([pt1, QPoint(pt1.x(), pt2.y()), pt2, QPoint(pt2.x(), pt1.y())])

    def drawObject(self, obj, painter, color):
        self.drawPoint(obj.location, painter, color)
        orientation_pt = obj.location + QPoint(ObjectFunction.ORIENTATION_LENGTH * math.cos(obj.orientation),
                                               ObjectFunction.ORIENTATION_LENGTH * math.sin(obj.orientation))
        painter.drawLine(obj.location, orientation_pt)

    def drawPoint(self, pt, painter, color):
        painter.setPen(color)
        painter.drawPoint(pt)
        painter.drawEllipse(pt, 3, 3)
