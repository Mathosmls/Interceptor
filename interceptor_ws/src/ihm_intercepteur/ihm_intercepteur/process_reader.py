#!/usr/bin/env python3

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

class ProcessReader(QObject):
    new_line = pyqtSignal(str)
    finished = pyqtSignal()

    def __init__(self, process):
        super().__init__()
        self.process = process
        self._running = True

    def stop(self):
        self._running = False

    def run(self):
        while self._running:
            line = self.process.stdout.readline()
            if not line:
                break
            self.new_line.emit(line.rstrip())

        self.finished.emit()