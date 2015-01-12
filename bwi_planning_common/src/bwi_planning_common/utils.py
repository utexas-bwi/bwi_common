#!/bin/env python

from python_qt_binding.QtGui import QLabel

def clearLayoutAndFixHeight(layout):
    # Clear all subfunction buttons.
    while layout.count():
        item = layout.takeAt(0)
        if item.widget() is not None:
            item.widget().deleteLater()
    height_label = QLabel()
    height_label.setFixedHeight(30)
    layout.addWidget(height_label)
