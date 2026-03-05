#!/usr/bin/env python3

import os

#Désactivation du GPU pour l'affichage de la carte (transforme le QWebEngine permettant l'affichage de Leaflet en widget QT classique, 
#ce qui permettra de respecter l'affichage des différents objets de la scène par couche)
#On force le QtWebEngine à ne PAS utiliser le GPU
#Sans cette ligne de code, QWebEngineView est rendu dans une surface GPU séparée qui passe au-dessus du moteur QGraphicsScene
os.environ["QTWEBENGINE_DISABLE_GPU"] = "1"
#Désactive le GPU au niveau du moteur Chromium embarqué dans le QtWebEngine
os.environ["QTWEBENGINE_CHROMIUM_FLAGS"] = "--disable-gpu"
#Force Qt Quick (QML / SceneGraph) en rendu logiciel
os.environ["QT_QUICK_BACKEND"] = "software"


#On force ROS à se lancer sur la jetson avec un ROS_DOMAIN_ID de 0, on autorise ROS a être difusé sur toutes les machines du réseau et on choisit le RMW
os.environ.setdefault("ROS_DOMAIN_ID", "0")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "0")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWebEngineWidgets import QWebEngineView
from threading import Thread
from ros_node_ihm import HMINode
from threading import Lock #Pour empêcher les interruptions lorsqu'on lit une valeur partagée entre le fil d'exécution de ROS et celui de Qt
import subprocess #Pour lancer le noeud mavros depuis l'IHM
import sys 
import glob #Pour la détection des ports USB / 
import signal #Pour détecter les "Ctrl+C" dans le terminal

#Imports des objets Qt que nous avons adaptés spécialement pour l'IHM
from qt_item import ModularQtItem
from zoomable_graphic_view import ZoomableGraphicsView
from scale_bar_widget import ScaleBarWidget
from leaflet_map_manager import LeafletMapManager
from process_reader import ProcessReader


import rclpy
import numpy as np

class HMIWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        #Création du Node ROS2 associée à l'IHM
        self.node_IHM = HMINode()

        self.setWindowTitle("IHM Mission - Intercepteur")
        self.resize(600, 600)

        #Création de la vue et de la scène
        self.view = ZoomableGraphicsView(self)
        self.scene = QGraphicsScene(self)
        self.view.setScene(self.scene)
        self.setCentralWidget(self.view)

        # Ajustement des dimensions de la scène (Le centre de la scène a pour coordonnées (0,0))
        self.scene.setSceneRect(-1800, -1800, 3600, 3600)

        #Style appliqué au combo boxes
        self.COMBOBOX_STYLE = """QComboBox {background-color: #1e1e1e;color: white;border: 1px solid #666;border-radius: 4px;padding: 4px;padding-right: 20px;font-size: 12px;}
        QComboBox:hover {border: 1px solid #888;}QComboBox:focus {border: 1px solid #5dade2;}
        QComboBox QAbstractItemView {background-color: #2b2b2b;color: white;selection-background-color: #5dade2;selection-color: black;outline: 0;}
        QComboBox::drop-down {subcontrol-origin: padding;subcontrol-position: top right;width: 16px;border-left: 1px solid #555;}
        QComboBox::down-arrow {image: none;border: none;}"""

        self.LABEL_STYLE = """QLabel {background-color: #1e1e1e;color: white;border: 1px solid #666;border-radius: 4px;padding: 6px;font-size: 12px;font-weight: bold;}QLabel:hover {border: 1px solid #888;}"""



        # ================= Création des items intercepteur et cible ================
        #Intercepteur (couleur bleue)
        # Création de l'objet intercepteur que l'on vient ajouter à la scène
        #Le repère de la scène étant celui du robot, les coordonnées de l'intercepteur dans ce repère sont donc (0,0) et son orientation 0°.
        self.interceptor_item = ModularQtItem(self.scene, name="intercepteur", color=(175,175,255)) 
        #On fait passer l'item au premier plan
        self.interceptor_item.item.setZValue(10)

        # Cible (couleur rouge)
        self.target_item = ModularQtItem(self.scene, name="cible", color=(255,175,175))
        #On fait passer l'item au premier plan
        self.target_item.item.setZValue(10)
        #Liste des items
        self.item_list = [self.interceptor_item, self.target_item]


        # ================= Attributs liés au zoom/pan =================
        # échelle en pixel/m 
        self.scale = 10
        self.zoom_factor = 1.0
        self.zoom_min = 0.2
        self.zoom_max = 20.0
        self.zoom_step = 1.15

        self.view.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.view.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.view.setDragMode(QGraphicsView.ScrollHandDrag)

        # Active la détection du pinch (pavé tactile)
        self.grabGesture(Qt.PinchGesture)

        
        # ================= Attributs liés à la carte OpenStreetMap =================
        #Longitude et latitude du centre de la scène (initialisée au centre du lac de Guerlédan). Ils seront modifiés lorsque l'on intègrera du pan (des translations)

        self.central_lat = 48.197717 #°
        self.central_long = -3.016452 #°
        self.alt_ref = 69 #m

        #Dernier zoom que l'on a appliqué à la carte
        self.last_map_zoom = None

        #Création du gestionnaire de carte (scene, lat/long du point de référence, échelle (en pixel/m))
        self.map_manager = LeafletMapManager( self.central_lat, self.central_long, self.scale)

        self.web_view = self.map_manager.web_view
        self.web_view.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.map_proxy = self.scene.addWidget(self.web_view)
        self.map_proxy.setZValue(-1000)


        # ================= Attributs liés au changement de référentiel =================
         #Repère de référence pour l'affichage : "WORLD" pour le repère du monde et "INTERCEPTOR" pour l'intercepteur.
        self.reference_frame = "WORLD"
        #Bouton permettant de changer de référentiel
        self.frame_button = self.init_frame_changing_button()


        # ================= Attributs liés au mode dans lequel se trouve le robot =================
        #Mode dans lequel se trouve le robot : "MANUAL" : mode manuel, "GUIDED" : mode guidé, ou None : on ne connaît pas le mode ou celui-ci n'est ni manuel, ni guidé
        self.mode = None  # "MANUAL", "GUIDED", ou None
        self.mode_button = self.init_mode_button()


        # ================= Attributs liés à l'armement/désarmement du robot =================
        #Booléen mis à True si le robot est armé, False si il est désarmé, None si on ne sait pas
        self.armed = None
        #Bouton permettant d'armer/désarmer le robot
        self.arm_button = self.init_arm_toggle_button()


        # ================= Bouton d'arrêt d'urgence ==================
        self.emergency_button = self.init_emergency_button()


        # ================= Attributs liés à l'affichage des images de la caméra (2 QLabels) =================
        #camera_label : label auquel on associe une QPixMap contenant les images de la caméra 
        #camear_title : titre des images de la caméra
        self.camera_label, self.camera_title = self.init_camera_item()


        # ================= Attributs liés à MAVROS =================
        #Création du bouton pour lancer le noeud mavros ainsi que des menus déroulants indiquant les paramètres du noeud
        self.port_combo, self.baud_combo, self.mavros_button, self.effecteur_combo = self.init_mavros_control()
        #Récupération et affichage de la liste des ports disponibles pour lancer mavros
        #self.refresh_serial_ports()
        #Proccessus permettant à mavros de tourner en parallèle
        self.mavros_process = None

        #Initialisation du "terminal" dans lequel on va venir afficher les messages de MAVROS
        self.terminal_toggle_button = self.init_ros_terminal()

        # ================ Attibuts liés à l'affichage des commandes de vitesse ===============
        self.init_velocity_display()

        # ================= Attributs liés à la barre d'échelle =================
        self.scale_bar = ScaleBarWidget(self.view)
        self.scale_bar.move(20, self.view.height() - 40)
        self.scale_bar.show()


        # ================= Crée et lance le Timer de mise à jour de la scène =================
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_scene)
        self.timer.start(1000)  #période en ms (20 Hz) 



#================================
#Fonction mettant à jour la carte 
#================================

    def update_map_view(self):
        view_rect = self.view.mapToScene(self.view.viewport().rect()).boundingRect()
        view_center = view_rect.center()
        self.map_manager.update_map(view_rect, self.view.zoom_factor, view_center)


#===============================================================================
#Fonction initialisant les boutons et les comboBoxes pour lancer le noeud mavros
#===============================================================================
    def init_mavros_control(self):
        """
        Initialise le menu MAVROS repliable (port / baudrate / start-stop)
        Placé sous le menu SSH
        """

        # ================= Bouton repliable =================
        self.mavros_toggle_button = QToolButton(self.view)
        self.mavros_toggle_button.setText("MAVROS ▸")
        self.mavros_toggle_button.setCheckable(True)
        self.mavros_toggle_button.setChecked(False)
        self.mavros_toggle_button.setFixedSize(180, 30)

        emergency_button = self.emergency_button.geometry()
        self.mavros_toggle_button.move(
            emergency_button.x(),
            emergency_button.y() + emergency_button.height() + 10
        )
        self.mavros_toggle_button.raise_()

        self.mavros_toggle_button.setStyleSheet("""
            QToolButton {
                background-color: #444;
                color: white;
                font-weight: bold;
                border-radius: 4px;
            }
        """)

        # ================= Conteneur repliable =================
        self.mavros_panel = QFrame(self.view)
        self.mavros_panel.setFrameShape(QFrame.StyledPanel)
        self.mavros_panel.setFixedSize(280, 230)
        self.mavros_panel.move(
            self.mavros_toggle_button.x() + self.mavros_toggle_button.width() + 20,
            self.mavros_toggle_button.y()
        )
        self.mavros_panel.setVisible(False)
        self.mavros_panel.raise_()

        self.mavros_panel.setStyleSheet("""
            QFrame {
                background-color: #2b2b2b;
                border: 1px solid #555;
                border-radius: 5px;
            }
        """)

        # ================= Layout =================
        layout = QVBoxLayout(self.mavros_panel)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        label_style = """
            QLabel {
                color: #cccccc;
                font-size: 11px;
                font-weight: bold;
            }
        """

        # =============== Type d'effecteur (rover ou intercepteur) ============
        effecteur_label = QLabel("Effecteur")
        effecteur_label.setStyleSheet(label_style)

        self.effecteur_combo = QComboBox()
        self.effecteur_combo.setStyleSheet(self.COMBOBOX_STYLE)
        self.effecteur_combo.addItems(["Rover","Intercepteur"])
        self.effecteur_combo.setCurrentText("Rover")
        self.effecteur_combo.setPlaceholderText("Rover")


        # ================= Port série =================
        port_label = QLabel("Port série")
        port_label.setStyleSheet(label_style)

        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setStyleSheet(self.COMBOBOX_STYLE)
        self.port_combo.addItems(["/dev/ttyUSB0","/dev/ttyUSB1"])
        self.port_combo.setCurrentText("/dev/ttyUSB0")
        self.port_combo.setPlaceholderText("/dev/ttyUSB0")

        # ================= Baudrate =================
        baud_label = QLabel("Baudrate")
        baud_label.setStyleSheet(label_style)

        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["57600", "115200", "921600"])
        self.baud_combo.setCurrentText("57600")
        self.baud_combo.setStyleSheet(self.COMBOBOX_STYLE)

        # ================= Bouton START / STOP =================
        action_label = QLabel("Contrôle")
        action_label.setStyleSheet(label_style)

        self.mavros_button = QPushButton("START MAVROS")
        self.mavros_button.setCheckable(True)
        self.mavros_button.clicked.connect(self.toggle_mavros)

        self.mavros_button.setStyleSheet("""
            QPushButton {
                background-color: #2E8B57;
                color: white;
                font-weight: bold;
                height: 28px;
            }
            QPushButton:checked {
                background-color: #B22222;
            }
        """)

        # ================= Ajout au layout =================
        layout.addWidget(effecteur_label)
        layout.addWidget(self.effecteur_combo)
        #layout.addSpacing(4)

        layout.addWidget(port_label)
        layout.addWidget(self.port_combo)
        #layout.addSpacing(6)

        layout.addWidget(baud_label)
        layout.addWidget(self.baud_combo)
        #layout.addSpacing(8)

        layout.addWidget(action_label)
        layout.addWidget(self.mavros_button)

        # ================= Toggle =================
        self.mavros_toggle_button.toggled.connect(self.toggle_mavros_panel)

        return self.port_combo, self.baud_combo, self.mavros_button, self.effecteur_combo

#================================================================
#Fonction appelée lorsque l'on clique sur le menu dépliant MAVROS
#================================================================

    def toggle_mavros_panel(self, checked):
        self.mavros_panel.setVisible(checked)
        self.mavros_toggle_button.setText("MAVROS ▾" if checked else "MAVROS ▸")

#======================================================================================================
#Fonction récupérant le nom des ports disponibles et les affichant dans le menu déroulant correspondant
#======================================================================================================

    def refresh_serial_ports(self):
        ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyTHS*")
        self.port_combo.clear()
        self.port_combo.addItems(ports if ports else ["Aucun port détecté"])

#==================================================================================
#Fonction demandant un lancement ou un arrêt du noeud mavros selon l'état du bouton
#==================================================================================

    def toggle_mavros(self, checked):
        if checked:
            self.start_mavros()
        else:
            self.stop_mavros()

#================================
#Fonction lançant le noeud mavros
#================================

    def start_mavros(self):
        self.node_IHM.get_logger().info(f"Effecteur demandé pour le lancement de ROS: {self.effecteur_combo.currentText()}.")
        if self.effecteur_combo.currentText() == "Rover":
            port = self.port_combo.currentText()
            baud = self.baud_combo.currentText()

            if "tty" not in port:
                QMessageBox.critical(self, "Erreur", "Port série invalide")
                self.mavros_button.setChecked(False)
                return

            # Commande pour lancer MAVROS avec un ROS_DOMAIN_ID de 0, la possibilité d'exporter les messages autre part que sur le local_host, et le middleware utilisé
            cmd = [
                "bash", "-c",
                f"export ROS_DOMAIN_ID=0; "
                f"export ROS_LOCALHOST_ONLY=0; "
                f"export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; "
                f"source /opt/ros/humble/setup.bash; "
                f"ros2 run mavros mavros_node --ros-args "
                f"--param fcu_url:={port}:{baud} "
                f"--param tgt_system:=1 "
                f"--param tgt_component:=1 "
                f"--param system_id:=255 "
                f"--param component_id:=191"
            ]

        elif self.effecteur_combo.currentText() == "Intercepteur":
            cmd = ["bash", "-c",
                f"export ROS_DOMAIN_ID=0; "
                f"export ROS_LOCALHOST_ONLY=0; "
                f"export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; "
                f"source /opt/ros/humble/setup.bash; "
                f"ros2 launch mavros apm.launch"
                f"--param fcu_url:=tcp://10.0.11.130:4003"]


        # Lance MAVROS et capture stdout/stderr
        if self.mavros_process is None:
            self.mavros_process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)

            self.reader = ProcessReader(self.mavros_process)
            self.reader_thread = QThread()
            self.reader.moveToThread(self.reader_thread)

            self.reader.new_line.connect(self.update_terminal_logs)
            self.reader.finished.connect(self.reader_thread.quit)

            self.reader_thread.started.connect(self.reader.run)
            self.reader_thread.start()

        self.mavros_button.setText("STOP MAVROS")
        self.mavros_button.setStyleSheet("background-color: #B22222; color: white;font-weight: bold;")



#=================================
#Fonction arrêtant le noeud mavros
#=================================

    def stop_mavros(self):
        #Arrêt dureader
        if hasattr(self, "reader") and self.reader:
            self.reader.stop()

        #Sortie propre du thread
        if hasattr(self, "reader_thread") and self.reader_thread:
            self.reader_thread.quit()
            self.reader_thread.wait()
            self.reader_thread = None
            self.reader = None

        #On arrête MAVROS
        if self.mavros_process:
            try:
                self.mavros_process.terminate()
                self.mavros_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.mavros_process.kill()
            finally:
                self.mavros_process = None


        self.mavros_button.setText("START MAVROS")
        self.mavros_button.setStyleSheet("background-color: #2E8B57; color: white;font-weight: bold;")


#========================================================================
#Fonction initialisant les panneaux d'affichages des commandes de vitesse
#========================================================================

    def init_velocity_display(self):
        """
        Initialise le panneau affichant les commandes en vélocité envoyées à MAVROS.
        Placé juste sous le toggle_button MAVROS.
        """
        # Création du frame
        self.velocity_panel = QFrame(self.view)
        self.velocity_panel.setFrameShape(QFrame.StyledPanel)
        self.velocity_panel.setFixedSize(200, 120)
        
        # Position juste sous le toggle_button MAVROS
        self.velocity_panel.move(
            self.mavros_toggle_button.x(),
            self.mavros_toggle_button.y() + self.mavros_toggle_button.height() + 10)

        self.velocity_panel.setStyleSheet("""
            QFrame {background-color: #2b2b2b; border: 1px solid #555; border-radius: 5px;}
        """)
        self.velocity_panel.setVisible(True)
        
        # Layout vertical pour les labels
        layout = QVBoxLayout(self.velocity_panel)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(4)
        
        label_style = """
            QLabel {color: #00ff00; font-family: monospace; font-size: 12px;}
        """
        
        #Label pour la vitesse de consigne
        self.title_vel_label = QLabel("Dans le repère du robot :")
        self.title_vel_label.setStyleSheet(self.LABEL_STYLE)
        layout.addWidget(self.title_vel_label)

        # Labels pour les vitesses linéaires (dans le repère du robot)
        self.vel_x_label = QLabel("V : 0.0 m/s")
        self.vel_x_label.setStyleSheet(label_style)
        layout.addWidget(self.vel_x_label)


        self.yaw_rate_label = QLabel("Omega : 0.0 deg/s")
        self.yaw_rate_label.setStyleSheet(label_style)
        layout.addWidget(self.yaw_rate_label)

#============================================================
#Fonction mettant à jour l'affichage des commandes en vitesse
#============================================================
    def update_velocity_display(self):
        """
        Met à jour les labels affichant la vélocité envoyée.
        """
        self.vel_x_label.setText(f"V : {self.node_IHM.V_r:.2f} m/s")
        self.yaw_rate_label.setText(f"Omega : {self.node_IHM.omega_r:.2f} deg/s")


#=============================================
#Fonction récupérant la liste des packages ROS
#=============================================

    def get_ros_packages():
        result = subprocess.check_output(["ros2", "pkg", "list"], text=True)
        return result.splitlines()

#===================================================================
#Fonction récupérant la liste fichiers launch du package en question
#===================================================================

    def get_launch_files(package):
        prefix = subprocess.check_output(
            ["ros2", "pkg", "prefix", package], text=True
        ).strip()
        launch_dir = Path(prefix) / "share" / package / "launch"

        if not launch_dir.exists():
            return []

        return [f.name for f in launch_dir.glob("*.py")]



#=======================================================
#Fonction initialisant le bouton de changement de repère
#=======================================================

    def init_frame_changing_button(self):
        frame_button = QPushButton("REPÈRE : MONDE", self)
        frame_button.setFixedSize(180,30)
        frame_button.clicked.connect(self.toggle_reference_frame)
        frame_button.setStyleSheet("""QPushButton {background-color: #4169E1;color: white;font-weight: bold;}""")

        # Position écran (en pixels, relatif à la vue)
        frame_button.move(10, 10)
        frame_button.raise_()

        return frame_button

#=====================================================================================
#Fonction permettant de changer de repère (associée au bouton de changement de repère)
#=====================================================================================

    def toggle_reference_frame(self):
        if self.reference_frame == "WORLD":
            self.reference_frame = "INTERCEPTOR"
            self.frame_button.setText("REPÈRE : INTERCEPTEUR")
            self.frame_button.setStyleSheet("""QPushButton {background-color: #4169E1;color: white;font-weight: bold;}""")
            self.node_IHM.get_logger().info(f"Passage au repère intercepteur.")
        else:
            self.reference_frame = "WORLD"
            self.frame_button.setText("REPÈRE : MONDE")
            self.frame_button.setStyleSheet("""QPushButton {background-color: #4169E1;color: white;font-weight: bold;}""")
            self.node_IHM.get_logger().info(f"Passage au repère du monde.")

#=====================================================
#Fontion créant le bouton pour armer/désarmer le robot
#=====================================================

    def init_arm_toggle_button(self):
        self.arm_button = QPushButton("ARMAMENT UNKNOWN", self.view)
        self.arm_button.setCheckable(True)
        self.arm_button.setChecked(False)

        self.arm_button.setFixedSize(180, 30)
        self.arm_button.move(10, 50)
        self.arm_button.raise_()

        self.arm_button.clicked.connect(self.on_arm_button_toggled)

        self.update_arm_button_style()

        return self.arm_button

#===========================================================================================================
#Fonction mettant à jour la couleur du bouton pour armer et désarmer le robot (rouge : armé, vert : désarmé)
#===========================================================================================================

    def update_arm_button_style(self):
        """
        armed : Mis à "True" si le robot est armé, mis à False si le robot est désarmé, mis à None si on ne sait pas si le robot est armé ou désarmé 
        """
        if self.armed == True:
            self.arm_button.setChecked(True)
            self.arm_button.setEnabled(True)
            self.arm_button.setText("ARMED")
            self.arm_button.setStyleSheet("""
                QPushButton {background-color: #B22222;color: white;font-weight: bold;}""")
        elif self.armed == False:
            self.arm_button.setChecked(False)
            self.arm_button.setEnabled(True)
            self.arm_button.setText("DISARMED")
            self.arm_button.setStyleSheet("""
                QPushButton {background-color: #2E8B57;color: white;font-weight: bold;}""")

        else:
            self.arm_button.setChecked(False)
            #On met setEnabled à False parce que l'on ne sait pas si le robot est armé ou non
            self.arm_button.setEnabled(False)
            self.arm_button.setText("ARMAMENT UNKNOWN")
            self.arm_button.setStyleSheet("""
                QPushButton {background-color:#808080;color: white;font-weight: bold;}""")

#=========================================================================================================================================================
#Fonction permettant au noeud ROS associé à l'IHM d'envoyer une demande d'armement / désarmement du robot (associé au bouton pour arme/rdésarmer le robot)
#=========================================================================================================================================================

    def on_arm_button_toggled(self, checked: bool):
        """
        checked = True  -> ARM
        checked = False -> DISARM
        """

        #Demande d'armement 
        if checked == True:
            self.node_IHM.arm_robot()
        #Demande de désarmement
        else:
            self.node_IHM.disarm_robot()
        return None


#======================================================
#Fontion créant le bouton pour changer le mode du robot
#======================================================

    def init_mode_button(self):
        self.mode_button = QPushButton("MODE UNKNOWN", self)
        self.mode_button.setCheckable(True)
        self.mode_button.setChecked(False)
        self.mode_button.setFixedSize(180, 30)
        self.mode_button.clicked.connect(self.on_mode_button_clicked)

        self.update_mode_button_style()

        self.mode_button.move(10, 90)
        self.mode_button.raise_()

        return self.mode_button


#======================================================================================================================================================
#Fonction permettant au noeud ROS associé à l'IHM d'envoyer une demande de changement de mode du robot (associé au bouton pour arme/rdésarmer le robot)
#======================================================================================================================================================

    def on_mode_button_clicked(self, checked: bool):
        """
        checked = True  → GUIDED
        checked = False → MANUAL
        """
        if checked:
            self.node_IHM.get_logger().error(f"Demande de passage en mode guidé.")
            self.node_IHM.set_guided_mode()
        else:
            self.node_IHM.get_logger().error(f"Demande de passage en mode manuel.")
            self.node_IHM.set_manual_mode()

#===========================================================================================================
#Fonction mettant à jour la couleur du bouton pour armer et désarmer le robot (rouge : armé, vert : désarmé)
#===========================================================================================================

    def update_mode_button_style(self):
        """
        mode : "MANUAL", "GUIDED" ou None
        """
        if self.mode == 'GUIDED':
            self.mode_button.setChecked(True)
            self.mode_button.setEnabled(True)
            self.mode_button.setText("MODE GUIDED")
            self.mode_button.setStyleSheet("""QPushButton {background-color: #4169E1;color: white;font-weight: bold;}""")

        elif self.mode == 'MANUAL':
            self.mode_button.setText("MODE MANUAL")
            self.mode_button.setChecked(False)
            self.mode_button.setEnabled(True)
            self.mode_button.setStyleSheet("""QPushButton {background-color: #2E8B57;color: white;font-weight: bold;}""")

        else:
            self.mode_button.setText("UNKNOWN MODE")
            self.mode_button.setChecked(False)
            self.mode_button.setStyleSheet("""QPushButton {background-color: #808080;color: white;font-weight: bold;}""")


#==================================
#Fontion créant le bouton EMERGENCY
#==================================
    def init_emergency_button(self):
        self.emergency_button = QPushButton("EMERGENCY", self)
        self.emergency_button.setFixedSize(180, 30)
        self.emergency_button.clicked.connect(self.on_emergency_button_clicked)
        self.emergency_button.setStyleSheet("""QPushButton {background-color: #B22222;color: white;font-weight: bold;}""")

        #Placement du bouton (juste sous le bouton de changement de mode)
        mode_geo = self.mode_button.geometry()
        self.emergency_button.move(mode_geo.x(), mode_geo.y() + mode_geo.height() + 10)
        self.emergency_button.raise_()

        return self.emergency_button


#====================================================================
#Fonction appelée lorsque l'on clique sur le bouton d'arrêt d'urgence
#====================================================================

    def on_emergency_button_clicked(self):
        self.node_IHM.disarm_robot()


#======================================================================================
#Fonction initialisant un item qui va nous permettre d'afficher les images de la caméra
#======================================================================================

    def init_camera_item(self):
        self.camera_label = QLabel(self)
        self.camera_label.setFixedSize(600, 350)
        self.camera_label.move(self.width()-620, 40)
        self.camera_label.setStyleSheet("border: 3px solid black;")
        self.camera_label.setScaledContents(True)
        self.camera_label.show()

        # Création d’un label contenant le titre que l'on placera au-dessus de l'image
        self.camera_title = QLabel("Caméra BASLER AC1920-50GC", self)
        font = QFont("Bahnschrift", 12, QFont.Bold)
        self.camera_title.setFont(font)
        self.camera_title.setStyleSheet("color: black;")
        self.camera_title.adjustSize()
        self.camera_title.show()
        return self.camera_label, self.camera_title


#=======================================
#Récupère la dernière image de la caméra
#=======================================
    def get_latest_image(self):
        """
        L'image récupérée est une QImage
        """
        with self.node_IHM.lock:
            if self.node_IHM.latest_image is None:
                #self.node_IHM.get_logger().info("L'image n'existe_pas")
                return None
            return self.node_IHM.latest_image


#===================================
#Fonction qui affiche un item camera
#===================================

    def update_camera_item(self):
        image = self.get_latest_image()
        if image is None:
            return
        #Pas besoin de redimensionner l'image, on a fixé les dimensions du label camera_label
        pixmap = QPixmap.fromImage(image)
        self.camera_label.setPixmap(pixmap)

#=====================================================================================================
#Fonction héritée de la classe QMainWindow, appelée la première fois que l'on doit afficher la fenêtre
#=====================================================================================================
    def showEvent(self, event):
        super().showEvent(event)
        #On met à jour l'affichage de la carte
        self.map_manager.show()
        self.map_manager.resize_view(self.view.viewport().width(), self.view.viewport().height())
        #Ajuster le proxy pour qu’il remplisse la scène
        self.map_proxy.setGeometry(self.scene.sceneRect())
        # #On réintiaialise la vue
        # self.reset_view()

#===========================================
#Fonction permettant de réinitialiser la vue
#===========================================

    def reset_view(self):
        self.view.resetTransform()
        self.zoom_factor = 1.0
        self.view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)


#==================================================================================================
#Fonction héritée de la classe QMainWindow, appelée à chaque fois que l'on redimensionne la fenêtre
#==================================================================================================

    def resizeEvent(self, event):
        super().resizeEvent(event)
        #self.view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)
        #Repositionnement de l'image reçue de la part de la caméra et du titre de l'image
        self.camera_label.move(self.width()-620, 40)
        self.camera_title.move(self.camera_label.x(), self.camera_label.y() - 25)  # juste au-dessus de l'image de la caméra

        #On repositionne le terminal
        camera_label_geo = self.camera_label.geometry()
        self.terminal_toggle_button.move(camera_label_geo.x(), camera_label_geo.y() + camera_label_geo.height() + 10)
        self.terminal_panel.move(self.terminal_toggle_button.x(), self.terminal_toggle_button.y() + self.terminal_toggle_button.height() + 5)

        #Replacement de la barre d'échelle
        self.scale_bar.move(20, self.view.height() - 40)

        #On met à jour l'affichage de la carte
        self.map_manager.resize_view(self.view.viewport().width(), self.view.viewport().height())

        #Ajuster le proxy pour qu’il remplisse la scène
        self.map_proxy.setGeometry(self.scene.sceneRect())


#===================================================================================================================================
#Fonction convertissant les données du repère du bateau au repère de la scène (coordonnées que l'on passe aux fonctions d'affichage)
#=================================================================================================================================== 

    def to_scene(self,pos, scale = 1):
        x = int(pos[0] * self.scale)
        y = int(-pos[1] * self.scale)
        return x, y

#====================================================================================================
#Fonction qui récupère les coordonnées et l'orientation d'un bateau en fonction du repère d'affichage 
#====================================================================================================

    def get_display_position_and_orientation(self, item):
        """
        :param item : item dont on veut connaître la position
        """
        if self.reference_frame == "WORLD":
            #Le moins est du au fait que les ordonnées 
            return item.position_NED[1,0], -item.position_NED[0,0], item.orientation_NED 

        elif self.reference_frame == "INTERCEPTOR":
            return item.position[0,0], item.position[1,0], item.orientation

#===================================================================
#Fonction qui affiche un item de type bateau (cible ou intercepteur)
#=================================================================== 

    def update_scene_ships(self, item):
        """
        Met à jour la représentation de l'item "item" (sa position, son orientation, etc...)
        """
        #self.node_IHM.get_logger().error(f"Position de l'item {item.name} : {item.position}")

        #Récupération des coordonnées de l'item dans le repère choisi 
        item_x, item_y, item_orientation = self.get_display_position_and_orientation(item)

        #Si la position de l'item n'est pas définie, on ne l'affiche pas
        if item_x == None or item_y == None:
            item.item.setVisible(False)
            self.node_IHM.get_logger().error(f"Impossible d'afficher l'item {item.name}. Position inconnue.")
            return None

        #Si les composantes de la position de l'item ne sont pas des entiers ou des flottants, on ne l'affiche pas
        if not (isinstance(item_x, (int, float, np.floating)) and isinstance(item_y, (int, float, np.floating))):
            item.item.setVisible(False)
            self.node_IHM.get_logger().error(f"Impossible d'afficher l'item {item.name}. Position : {item_x,item_y}")
            return None

        #Les coordonnées de l'item sont définies. On rend l'item visible.
        item.item.setVisible(True)

        #On ne connaît pas l'orientation de l'item, on la définit à 0° (Ce qui importe pour l'interception, c'est la position de la cible, pas son orientation)
        if item_orientation == None or not isinstance(item_orientation, (int, float)):
            self.node_IHM.get_logger().error(f"Orientation de l'item '{item.name}' inconnue.")
            item_orientation = 0
        #On garde l'item à la même échelle peu importe le zoom
        item.item.setScale(5/self.view.zoom_factor)

        #Met à jour la position de l'item (le moins vient du fait que les y positifs du repère du bateau sont négatifs dans le repère de la scène)
        #Ref_bateau = (x,y), Ref_scène = (x,-y)
        item.item.setPos(item_x*self.scale, -item_y*self.scale)
        #self.node_IHM.get_logger().error(f"Affichage de  l'item {item.name}. Position : {item_x*self.scale, -item_y*self.scale}")

        #Met à jour l'orientation de l'item (dans le sens des aiguilles d'une montre, d'où l'abscence de moins) 
        #self.node_IHM.get_logger().info(f"Orientation de l'item '{item.name}' : {item_orientation}.")
        item.item.setRotation(90-item_orientation)


#===========================================================
#Met à jour la position des items de la scène et les affiche
#===========================================================
    def update_scene(self):
        #On utilise le bloqueur d'interruptions du node ROS2 pour empêcher les interruptions lorsque l'on met à jour
        #On récupère les données des topics ROS2 (la position et l'orientation de la cible et de l'intercepteur, les images de la caméra)
        with self.node_IHM.lock:
            ##Pour l'intercepteur : position et orientation dans le repère NED du monde et dans le repère de l'intercepteur
            self.interceptor_item.position_NED = self.node_IHM.pos_intercepteur_NED #tableau numpy de dimensions (3,1)
            self.interceptor_item.orientation_NED = self.node_IHM.orientation_intercepteur_NED # float 
            self.interceptor_item.position = self.node_IHM.pos_intercepteur #tableau numpy de dimensions (3,1)
            self.interceptor_item.orientation = self.node_IHM.orientation_intercepteur # float 
            #self.node_IHM.get_logger().error(f"Nouvelle position de l'intercepteur : {self.interceptor_item.position[1,0], -self.interceptor_item.position[0,0]}.")

            ##Pour la cible : position et orientation dans le repère NED du monde et dans le repère de l'intercepteur
            self.target_item.position_NED = self.node_IHM.pos_cible_NED
            self.target_item.orientation_NED = self.node_IHM.orientation_cible_NED
            self.target_item.position = self.node_IHM.pos_cible
            self.target_item.orientation = self.node_IHM.orientation_cible
            #self.node_IHM.get_logger().error(f"Nouvelle position de la cible : {self.target_item.position[1,0], -self.target_item.position[0,0]}.")

            ##Mise à jour de l'armement du bateau
            #On récupère l'état d'armement du robot (armé/désarmé)
            self.armed = self.node_IHM.armed
            self.update_arm_button_style()

            ##Mise à jour du mode du robot
            #On récupère le mode du robot (armé/désarmé)
            self.mode = self.node_IHM.mode
            #Mise à jour de l'état du bouton
            self.update_mode_button_style()

            #On met à jour l'affichage des commandes de vitesse
            self.update_velocity_display()

        #Dessine l'image reçue de la part de la caméra
        self.update_camera_item()

        #Dessine les items à leurs nouvelles positions (intercepteur, cible)
        for item in self.item_list:
            self.update_scene_ships(item)


#==============================================================
#Fonction mettant à jour la barre d'échelle en fonction du zoom
#==============================================================

    def update_scale_bar(self):
        zoom = self.view.zoom_factor

        # Choix intelligent de l’échelle
        if zoom < 0.5:
            meters = 20
        elif zoom < 1:
            meters = 10
        elif zoom < 3:
            meters = 5
        else:
            meters = 1

        self.scale_bar.set_meters(meters)

        # Repositionnement dans le repère de la fenêtre
        self.scale_bar.move(20, self.view.height() - 40)


#=================================================================================================================
#Fonction initialisant le terminal où sont affiché les messages qu'afficherait MAVROS sur le terminal de la jetson
#=================================================================================================================

    def init_ros_terminal(self):
        # Bouton pour afficher/cacher le terminal
        self.terminal_toggle_button = QToolButton(self.view)
        self.terminal_toggle_button.setText("TERMINAL ROS ▸")
        self.terminal_toggle_button.setCheckable(True)
        self.terminal_toggle_button.setFixedSize(180, 30)

        camera_label_geo = self.camera_label.geometry()
        self.terminal_toggle_button.move(camera_label_geo.x(), camera_label_geo.y() + camera_label_geo.height() + 10)
        self.terminal_toggle_button.raise_()
        self.terminal_toggle_button.toggled.connect(self.toggle_ros_terminal)

        self.terminal_toggle_button.setStyleSheet("""
            QToolButton {
                background-color: #444;
                color: white;
                font-weight: bold;
                border-radius: 4px;
            }
        """)

        # Panel terminal
        self.terminal_panel = QFrame(self.view)
        self.terminal_panel.setFrameShape(QFrame.StyledPanel)
        self.terminal_panel.setFixedSize(600, 200)
        self.terminal_panel.move(self.terminal_toggle_button.x(),
                                 self.terminal_toggle_button.y() + self.terminal_toggle_button.height() + 5)
        self.terminal_panel.setVisible(False)
        self.terminal_panel.setStyleSheet("""
            QFrame {background-color: #1e1e1e; border: 1px solid #555; border-radius: 5px;}
        """)

        # Layout pour terminal
        layout = QVBoxLayout(self.terminal_panel)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(2)

        # QTextEdit pour afficher les logs
        self.terminal_text = QTextEdit()
        self.terminal_text.setReadOnly(True)
        self.terminal_text.setStyleSheet("""
            QTextEdit {background-color: black; color: white; font-family: monospace; font-size: 12px;}
        """)
        layout.addWidget(self.terminal_text)

        return self.terminal_toggle_button



#====================================================================================
#Fonction appelée lorsque l'on clique sur le terminal (pour l'étendre / le rétracter)
#====================================================================================

    def toggle_ros_terminal(self, checked):
        self.terminal_panel.setVisible(checked)
        self.terminal_toggle_button.setText("TERMINAL ROS ▾" if checked else "TERMINAL ROS ▸")


#=================================================================================
#Fonction lisant les logs MAVROS avant de les afficher dans le "terminal" de l'IHM
#=================================================================================

    def update_terminal_logs(self,text):
        self.terminal_text.append(text.rstrip())
        self.terminal_text.moveCursor(QTextCursor.End)



def main():
    rclpy.init()

    app = QApplication(sys.argv)
    window = HMIWindow()
    window.show()

    #Crée un thread pour rclpy.spin qui est une fonction bloquante. Cette fonction sera donc exécutée en parallèle du programme;
    ros_thread = Thread(target=rclpy.spin, args=(window.node_IHM,), daemon=True)
    ros_thread.start()

    exit_code = app.exec_()

    #Si on reçoit ces signaux ("Ctrl+c" ou "kill" dans le terminal), on arrête l'application Qt
    signal.signal(signal.SIGINT, lambda s, f: app.quit()) # Ctrl+C
    signal.signal(signal.SIGTERM, lambda s, f: app.quit()) # kill

    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()