#!/usr/bin/env python3

import paramiko
import threading
from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

class SSHPasswordDialog(QDialog):
    def __init__(self, host="None", username="None", parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"Connexion SSH à {host}")
        self.password = None

        layout = QVBoxLayout(self)
        layout.addWidget(QLabel(f"Utilisateur : {username}"))
        layout.addWidget(QLabel(f"Hôte : {host}"))

        self.password_input = QLineEdit()
        self.password_input.setEchoMode(QLineEdit.Password)
        layout.addWidget(self.password_input)

        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)

    def get_password(self):
        return self.password_input.text()
