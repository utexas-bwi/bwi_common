#!/bin/env python

from python_qt_binding.QtCore import QPoint, QPointF, QSize
from python_qt_binding.QtGui import QLabel, QPolygon

def clearLayoutAndFixHeight(layout):
    # Clear all subfunction buttons.
    while layout.count():
        item = layout.takeAt(0)
        if item.widget() is not None:
            item.widget().deleteLater()
    height_label = QLabel()
    height_label.setFixedHeight(30)
    layout.addWidget(height_label)
    
def determineScale(orig_size, new_size):
    if orig_size.width() > orig_size.height():
        scale = float(new_size.width()) / float(orig_size.width())
    else:
        scale = float(new_size.height()) / float(orig_size.height())
    return scale

def scalePoint(pt, orig_size, new_size):
    scale = determineScale(orig_size, new_size)
    return scale * pt

def scalePolygon(polygon, orig_size, new_size):
    ret_poly = QPolygon()
    for point_idx in range(polygon.size()):
        point = polygon.point(point_idx)
        scaled_point = scalePoint(point, orig_size, new_size)
        ret_poly.append(scaled_point)
    return ret_poly

def transformPointToPixelCoordinates(pt, map, image_size):
    map_point_f = (pt - QPointF(map.map.info.origin.position.x, map.map.info.origin.position.y)) * (1.0 / map.map.info.resolution)
    map_point = QPoint(int(map_point_f.x()), int(map_point_f.y()))
    map_size = QSize(map.map.info.width, map.map.info.height)
    return scalePoint(map_point, map_size, image_size) 

def transformPointToRealWorldCoordinates(pt, map, image_size):
    map_size = QSize(map.map.info.width, map.map.info.height)
    map_point = scalePoint(pt, image_size, map_size)
    return QPointF(map.map.info.origin.position.x, map.map.info.origin.position.y) + \
            QPointF(map_point) * map.map.info.resolution 