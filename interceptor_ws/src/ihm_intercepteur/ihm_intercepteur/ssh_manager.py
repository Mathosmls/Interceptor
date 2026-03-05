#!/usr/bin/env python3

import paramiko
import threading
from PyQt5.QtCore import QObject, pyqtSignal

class SSHManager(QObject):
    output_received = pyqtSignal(str)
    error_received = pyqtSignal(str)
    connection_lost = pyqtSignal()

    def __init__(self, host="", username="", password="", port=22):
        super().__init__()
        self.host = host
        self.username = username
        self.password = password
        self.port = port
        self.client = None
        self.channel = None
        self.connected = False

    def connect(self):
        try:
            self.client = paramiko.SSHClient()
            self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            self.client.connect(self.host, username=self.username, password=self.password, port=self.port)
            self.connected = True
            return True
        except Exception as e:
            print(f"Erreur SSH : {e}")
            return False

    def disconnect(self):
        if self.client:
            self.client.close()
            self.connected = False

    def run_command(self, command):
        if not self.connected:
            raise RuntimeError("SSH non connecté")
        
        self.channel = self.client.get_transport().open_session()
        self.channel.exec_command(command)

        def read_output():
            try:
                while True:
                    if self.channel.recv_ready():
                        out = self.channel.recv(1024).decode()
                        self.output_received.emit(out)
                    if self.channel.recv_stderr_ready():
                        err = self.channel.recv_stderr(1024).decode()
                        self.error_received.emit(err)
                    if self.channel.exit_status_ready():
                        break
                #self.disconnect()
                #self.connection_lost.emit()
            except Exception as e:
                self.error_received.emit(f"Erreur lecture SSH: {e}")
                self.disconnect()
                self.connection_lost.emit()

        threading.Thread(target=read_output, daemon=True).start()

    def is_connected(self):
        return self.connected and self.client.get_transport().is_active()
