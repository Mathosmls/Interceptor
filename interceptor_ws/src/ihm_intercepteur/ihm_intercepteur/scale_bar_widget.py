#!/usr/bin/env python3

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from rclpy.node import Node



class ScaleBarWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.meters = 10
        self.pixels_per_meter = 10
        self.bar_height = 4

        self.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.setFixedHeight(30)

    def set_meters(self, meters):
        self.meters = meters
        self.updateGeometry()
        self.update()

    def sizeHint(self):
        #On récupère le facteur de zoom actuel
        view = self.parent()
        zoom = getattr(view, "zoom_factor", 1.0)

        bar_length = self.meters * self.pixels_per_meter * zoom
        return QSize(int(bar_length + 200), 40)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        #On récupère le facteur de zoom actuel
        view = self.parent()
        zoom = getattr(view, "zoom_factor", 1.0)

        bar_length = self.meters * self.pixels_per_meter *zoom
        y = self.height() // 2

        painter.setPen(Qt.NoPen)
        painter.setBrush(Qt.black)

        painter.drawRect(QRectF(10, y+6-self.bar_height / 2, bar_length, self.bar_height))

        painter.setPen(Qt.black)
        painter.drawText(10, y, f"{self.meters} m")#, {bar_length}, zoom {zoom}")

