#!/usr/bin/env python3

import sys
import numpy as np
import rclpy
from PyQt5.QtGui import QImage
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
import threading #Pour tout ce qui touche aux threads et aux interruptions
#from ihm_intercepteur.msg import *

"""
Vérification de la configuration du GPS : 
- config/sbg_device_uart_default
"""

class HMINode(Node):
    def __init__(self):
        super().__init__('boat_IHM')
        #Latitude, longitude et altitude de référence du repère NED (point situé en face des pontons à Guerlédan)
        self.lat_ref = 48.197717 #degrés
        self.long_ref = -3.016452 #degrés
        self.alt_ref = 69 #m

        #Position et orientation de l'intercepteur dans son repère 
        self.pos_intercepteur = np.array([[None],[None],[None]], np.float64)
        self.orientation_intercepteur = 0

        #Position et orientation de la cible dans le repère de l'intercepteur
        self.pos_cible = np.array([[None],[None],[None]], np.float64)
        self.orientation_cible = None

        #Position et orientation de l'intercepteur (dans le repère NED dont l'origine se trouve en face des pontons à Guerlédan)
        self.pos_intercepteur_NED = np.array([[None],[None],[None]], np.float64)
        self.orientation_intercepteur_NED = None

        #Position et orientation de l'intercepteur (dans le repère NED dont l'origine se trouve en face des pontons à Guerlédan)
        self.pos_cible_NED = np.array([[None],[None],[None]], np.float64)
        self.orientation_cible_NED = None

        #Image reçue de la caméra (image numpy BGR)
        self.latest_image = None
        #Image traitée par YOLO 
        self.latest_yolo_image = None 

        #On considère que YOLO n'est pas lancé, ce paramètre se mettra à True si on reçoit une image valide de la part de YOLO
        self.yolo_fonctionnel = False
        #Nombre de périodes du timer où l'on a pas reçu d'images traitées par YOLO
        self.last_period_yolo_received = 0

        ## ## Subscription permettant d'obtenir l'orientation et les coordonnées (lat, long, alt) (type du message, nom du topic, fonction de callback,10)
        #Orientation de l'intercepteur (angles d'Euler)
        #!!! Le GPS est en convention NED (x=Nord, y=Est, z=Bas)!!!
        self.create_subscription(Float64,'/mavros/global_position/compass_hdg', self.orientation_intercepteur_callback,10)

        self.create_subscription(NavSatFix,'/mavros/global_position/global', self.pose_intercepteur_callback,10)

        ## ## Subscription permettant d'obtenir la pose de la cible (type du message, nom du topic, fonction de callback,10)
        #Orientation de l'intercepteur (angles d'Euler)
        #!!! Le GPS est en convention NED (x=Nord, y=Est, z=Bas)!!!
        self.create_subscription(Float64,'/mavros/global_position/compass_hdg_cible', self.orientation_cible_callback,10)

        self.create_subscription(NavSatFix,'/mavros/global_position/global_cible', self.pose_cible_callback,10)

        ## ## Souscription permettant de récupérer la valeur des commandes en vitesse liénaire et angulaire envoyées aux moteurs
        self.create_subscription(Twist,'/mavros/setpoint_velocity/cmd_vel_unstamped', self.cmd_vel_callback,10)


        ## ## Subscription permettant de récupérer les images en couleur de la caméra Pylon
        self.create_subscription(Image,'/image_raw', self.image_callback,10)#image/image_raw


        ## ## Subscription permettant de récuprérer les images avec boîtes d'ancrage fournies par YOLO
        self.create_subscription(Image,'/detect/image_raw', self.image_yolo_callback,10)

        ## ## Subscription permettant de récupérer l'état de MAVROS
        self.create_subscription(State,'/mavros/state', self.mavros_state_callback,10)

        timer_period = 1/10 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #Client demandant à mavros de s'armer ou de se désarmer (pour chaque requête il reçoit un booléen indiquant s'il a réussi à le faire ou non)
        self.arm_client = self.create_client(CommandBool,'/mavros/cmd/arming')

        #Client demandant à mavros de changer de mode      
        self.mode_client = self.create_client(SetMode, "/mavros/cmd/set_mode")

        #Paramètre mis à True si le robot est armé, à False si il est désarmé et à None si on ne reçoit pas d'informations de la part de MAVROS
        self.armed = None

        #Mode dans lequel se trouve le robot. Si ce mode n'est pas 'GUIDED' ou 'MANUAL' ou que l'on ne connaît pas le mode, on met le mode à None
        self.mode = None

        #Vitesses de consignes 
        self.V_r = 0.0
        self.omega_r = 0.0

        #Dictionnaire dans lequel on vient stocker en entrée le nom des objets que l'on veut dessiner avec l'IHM (clé)
        # et un entier que l'on met à 1 à chaque fois que l'on modifie la position ou l'orientation de l'objet (valeur).
        #Cette valeur est remise à 0 lorsque l'on a actualisé la position de l'objet en question
        self.changement_effectues = {}

        # Création d'un objet empêchant les interruptions durant la lecture des variables partagées avec le 
        self.lock = threading.Lock()

#======================================================================================
#Fonction convertissant des données (latitude, longitude, altitude) dans le repère ECEF
#======================================================================================

    def geodetic_to_ecef(self,lat, lon, alt):
        """
        :param lat: latitude (en degrés)
        :param lon: longitude (en degrés)
        :param alt: altitude (en mètres)

        :return : coordonnées (x,y,z) dans le repère ECEF (tableau numpy de dimension (1,3))
        """
        lat = np.deg2rad(lat)
        lon = np.deg2rad(lon)

        a = 6378137.0
        f = 1 / 298.257223563
        e2 = f * (2 - f)

        N = a / np.sqrt(1 - e2 * np.sin(lat)**2)

        X = (N + alt) * np.cos(lat) * np.cos(lon)
        Y = (N + alt) * np.cos(lat) * np.sin(lon)
        Z = (N * (1 - e2) + alt) * np.sin(lat)

        return np.array([X, Y, Z])

#========================================================================
#Fonction convertissant des coodronnées du repère ECEF dans le repère NED
#========================================================================

    def ecef_to_ned(self, ecef, ecef_ref, lat_ref, lon_ref):
        """
        :param lat_ref: latitude de référence (en degrés)
        :param lon: longitude de référence (en degrés)

        :return : coordonnées (x,y,z) dans le repère NED (tableau numpy de dimension (1,3)) (X:Nord, Y:Est, Z:Bas)
        """
        lat_ref = np.deg2rad(lat_ref)
        lon_ref = np.deg2rad(lon_ref)

        dx = ecef - ecef_ref

        R = np.array([
            [-np.sin(lat_ref)*np.cos(lon_ref), -np.sin(lat_ref)*np.sin(lon_ref),  np.cos(lat_ref)],
            [-np.sin(lon_ref),                 np.cos(lon_ref),                  0],
            [-np.cos(lat_ref)*np.cos(lon_ref), -np.cos(lat_ref)*np.sin(lon_ref), -np.sin(lat_ref)]])

        ned = R @ dx
        return ned  # [N, E, D] en m

#=============================================================================================
#Fonction sawtooth pour obtenir l'angle signé le plus petit lorsque l'on soustrait deux angles
#=============================================================================================

    def sawtooth(self,x):
        return (x+np.pi)%(2*np.pi)-np.pi   # or equivalently   2*arctan(tan(x/2))

#========================================================
#Fonction calculant une matrice de rotation d'angle theta
#========================================================

    def compute_rotation_matrix(self, theta):
        """
        Renvoie la matrice de rotation d'angle théta autour de l'axe z
        """
        theta_rad = np.deg2rad(theta)
        return np.array([[np.cos(theta_rad),-np.sin(theta_rad),0],[np.sin(theta_rad),np.cos(theta_rad),0],[0,0,1]])

#=================================================================================================
#Fonction appelée lorsque l'on reçoit un nouveau message contenant l'orientation de l'intercepteur
#=================================================================================================

    def orientation_intercepteur_callback(self, msg):
        #self.get_logger().info("Réception de l'orientation de l'intercepteur")
        #On récupère le cap du robot en degrés (0°=>N, 90°=>E, 180°=>S, 270°=>O) et on la stocke
        #On bloque les interruptions, dont celles de Qt qui pourraient avoir accès aux variables que l'on modifie
        with self.lock:
            self.orientation_intercepteur_NED = msg.data
            #On indique grâce au dictionnaire que l'on a une nouvelle donnée d'orientation de l'intercepteur.
            #self.get_logger().info(f"Orientation de l'intercepteur : {self.orientation_intercepteur_NED}")
            self.changement_effectues["orientation_intercepteur"] = 1

#===============================================================================================
#Fonction appelée lorsque l'on reçoit un nouveau message contenant la position de l'intercepteur
#===============================================================================================

    def pose_intercepteur_callback(self, msg):
        #self.get_logger().info("Réception de la pose de l'intercepteur")
        ##Si la position GPS n'a pas pu être déterminée, on sort de la fonction
        #La ligne de code suivante est équivalent à "msg.status.status = NavSatStatus.STATUS_NO_FIX:
        if msg.status.status == -1:
            return
        ## ## On récupère la latitude, la longitude et l'altitude de l'intercepteur et on la convertit dans le repère NED du lac de Guerlédan
        latitude, longitude, altitude = msg.latitude, msg.longitude, msg.altitude 
        ecef_coordinates = self.geodetic_to_ecef(latitude, longitude, altitude)
        ecef_ref = self.geodetic_to_ecef(self.lat_ref, self.long_ref, self.alt_ref)
        ned_coordinates = self.ecef_to_ned(ecef_coordinates, ecef_ref, self.lat_ref, self.long_ref)
        #Mise à jour de la position de l'intercepteur dans le repère NED
        #On bloque les interruptions, dont celles de Qt qui pourraient avoir accès aux variables que l'on modifie
        with self.lock:
            self.pos_intercepteur_NED = np.array([[ned_coordinates[0]],[ned_coordinates[1]],[ned_coordinates[2]]])
            #On indique grâce au dictionnaire que l'on a une nouvelle donnée de pose de l'intercepteur.
            #self.get_logger().info(f"Pose de l'intercepteur : {self.pos_intercepteur_NED}")
            self.changement_effectues["pose_intercepteur"] = 1

#===========================================================================================
#Fonction appelée lorsque l'on reçoit un nouveau message contenant l'orientation de la cible
#===========================================================================================

    def orientation_cible_callback(self, msg):   
        #On récupère l'orientation de la cible dans le repère NED en radians, on convertit cette orientation en degrés
        #on convertit cette orientation dans le repère du robot
        #On bloque les interruptions, dont celles de Qt qui pourraient avoir accès aux variables que l'on modifie
        with self.lock:
            self.orientation_cible_NED = msg.data
            #Conversion dans le repère du robot
            self.orientation_cible = np.rad2deg(self.sawtooth(np.deg2rad(self.orientation_cible_NED - self.orientation_intercepteur)))
            self.changement_effectues["orientation_cible"] = 1

#=====================================================================================
#Fonction appelée lorsque l'on reçoit un nouveau message contenant la pose de la cible
#=====================================================================================
    def pose_cible_callback(self, msg):
        ## ## On récupère la latitude, la longitude et l'altitude de la cible et on la convertit dans le repère NED du lac de Guerlédan
        latitude, longitude, altitude = msg.latitude, msg.longitude, msg.altitude 
        ecef_coordinates_cible = self.geodetic_to_ecef(latitude, longitude, altitude)
        ecef_ref = self.geodetic_to_ecef(self.lat_ref, self.long_ref, self.alt_ref)
        ned_coordinates_cible = self.ecef_to_ned(ecef_coordinates_cible, ecef_ref, self.lat_ref, self.long_ref)
        #On indique grâce au dictionnaire que l'on a une nouvelle donnée d'orientation de la cible
        #On bloque les interruptions, dont celles de Qt qui pourraient avoir accès aux variables que l'on modifie
        with self.lock:
            self.pos_cible_NED = ned_coordinates_cible.reshape((3,1))
            #Si l'orientation de l'intercepteur est inconnue, on prend la matrice identité
            if self.orientation_intercepteur_NED == None:
                R_w_i = np.eye(3)
            else:
                #Matrice de rotation du repère NED au repère de l'intercepteur(z_NED = z_Intercepteur)
                R_w_i = self.compute_rotation_matrix(self.orientation_intercepteur_NED)
            ## ##Conversion de la position de la cible dans le repère de l'intercepteur
            #Vecteur de translation du repère NED au repère de l'intercepteur
            if self.pos_intercepteur_NED[0,0] == None:
                t_w_i = np.zeros((3,1))
            else:
                t_w_i = -self.pos_intercepteur_NED
            self.pos_cible = R_w_i@self.pos_cible_NED + t_w_i  
            #On indique grâce au dictionnaire que l'on a une nouvelle donnée de pose de la cible
            #self.get_logger().info(f"Pose de la cible : {self.pos_cible}")
            self.changement_effectues["position_cible"] = 1   


#===============================================================================================================
#Fonction appelée lorsque l'on récupère un message contenant la consigne de commande en vitesse envoyée à MAVROS
#===============================================================================================================

    def cmd_vel_callback(self, message: Twist):
        #Vitesses de consigne dans le repère du robot
        V_r, omega_z_r = message.linear.x, message.angular.z

        self.V_r = V_r
        self.omega_r = omega_r

        return None

#=====================================================================
#Fonction stockant les images publiées sur le topic "/image/image_raw"
#=====================================================================

    def image_callback(self, message: Image):
        w = message.width
        h = message.height
        step = message.step
        data = message.data
        if message.encoding == 'rgb8':
            qimage = QImage(data, w, h, step, QImage.Format_RGB888)

        elif message.encoding == 'bgr8':
            img = QImage(data, w, h, step, QImage.Format_BGR888)
            qimage = img

        elif message.encoding == 'mono8':
            qimage = QImage(data, w, h, step, QImage.Format_Grayscale8)

        elif message.encoding == 'rgba8':
            qimage = QImage(data, w, h, step, QImage.Format_RGBA8888)

        else:
            raise ValueError(f"Encoding ROS non supporté : {message.encoding}")
            return

        if qimage.isNull():
            self.get_logger().warn("QImage invalide.")
            return

        self.get_logger().info("Image reçue")
        if not self.yolo_fonctionnel:
            self.get_logger().info("Image stockée")
            self.latest_image = qimage


#=====================================================================
#Fonction stockant les images publiées sur le topic "/image/image_raw"
#=====================================================================

    def image_yolo_callback(self, message: Image):
        """
        Stocke les images sous forme de QImage
        """
        w = message.width
        h = message.height
        step = message.step
        data = message.data

        if message.encoding == 'rgb8':
            qimage = QImage(data, w, h, step, QImage.Format_RGB888)

        elif message.encoding == 'bgr8':
            message = QImage(data, w, h, step, QImage.Format_BGR888)
            qimage = img

        elif message.encoding == 'mono8':
            qimage = QImage(data, w, h, step, QImage.Format_Grayscale8)

        elif message.encoding == 'rgba8':
            qimage = QImage(data, w, h, step, QImage.Format_RGBA8888)

        else:
            raise ValueError(f"Encoding ROS non supporté : {message.encoding}")
            return

        if qimage.isNull():
            self.get_logger().warn("QImage invalide.")
            return

        #YOLO est fonctionnel, on l'indique
        self.yolo_fonctionnel = True
        #On remet à 0 le nombre de périodes du timer où l'on a pas reçu d'images traitées par YOLO
        self.last_period_yolo_received = 0
        self.latest_image = qimage

#=================================================
#Fonction envoyant à MAVROS une requête d'armement
#=================================================

    def arm_robot(self):
        #On vérifie si le serveur est joignable
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Service d'armement indisponible")
            return

        req = CommandBool.Request()
        req.value = True

        #Le client envoie la requête grâce à la fonction non bloquante "call_async".
        #Cette fonction renvoie un conteneur "future" qui sera rempli lorsque le serveur aura envoyé sa réponse au client
        future = self.arm_client.call_async(req)
        #On ajoute une fonction à appeler quand le conteneur "future" sera rempli
        future.add_done_callback(self.arm_response_callback)


#=====================================================
#Fonction envoyant à MAVROS une requête de désarmement
#=====================================================

    def disarm_robot(self):
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Service de désarmement indisponible")
            return

        req = CommandBool.Request()
        req.value = False

        #Le client envoie la requête grâce à la fonction non bloquante "call_async".
        #Cette fonction renvoie un conteneur "future" qui sera rempli lorsque le serveur aura envoyé sa réponse au client
        future = self.arm_client.call_async(req)
        #On ajoute une fonction à appeler quand le conteneur "future" sera rempli
        future.add_done_callback(self.arm_response_callback)


#===================================================================
#Fonction affichant le résultat de la requête d'armement/désarmement
#===================================================================

    def arm_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Commande ARM/DISARM acceptée par MAVROS")
            else:
                self.get_logger().warn("Commande ARM/DISARM refusée par MAVROS")
        except Exception as e:
            self.get_logger().error(f"Erreur service arming : {e}")


#============================================================
#Fonction transmettant la requête de passage en mode "GUIDED"
#============================================================

    def set_guided_mode(self):
        req = SetMode.Request()
        req.custom_mode = "GUIDED"
        self.get_logger().info(f"Demande de passage en mode guidé.")
        #Le client envoie la requête grâce à la fonction non bloquante "call_async".
        #Cette fonction renvoie un conteneur "future" qui sera rempli lorsque le serveur aura envoyé sa réponse au client
        future = self.mode_client.call_async(req)
        #On ajoute une fonction à appeler quand le conteneur "future" sera rempli
        future.add_done_callback(self.mode_response_callback)

#============================================================
#Fonction transmettant la requête de passage en mode "MANUAL"
#============================================================

    def set_manual_mode(self):
        req = SetMode.Request()
        req.custom_mode = "MANUAL"
        self.get_logger().info(f"Demande de passage en mode manuel.")
        #Le client envoie la requête grâce à la fonction non bloquante "call_async".
        #Cette fonction renvoie un conteneur "future" qui sera rempli lorsque le serveur aura envoyé sa réponse au client
        future = self.mode_client.call_async(req)
        #On ajoute une fonction à appeler quand le conteneur "future" sera rempli
        future.add_done_callback(self.mode_response_callback)


#==================================================================
#Fonction affichant le résultat de la requête de changement de mode
#==================================================================

    def mode_response_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent == True:
                self.get_logger().info("Commande de changement de mode acceptée par MAVROS")
            else:
                self.get_logger().warn("Commande de changement de mode refusée par MAVROS")
        except Exception as e:
            self.get_logger().error(f"Erreur service arming : {e}")


#==============================================================================================================
#Fonction appelée par le timer pour vérifier si on a reçu une image traitée par YOLO il y a moins d'une seconde
#==============================================================================================================

    def timer_callback(self):
        #On n'a pas reçu d'image traitée par YOLO depuis 10 périodes 
        if self.last_period_yolo_received > 10:
            self.yolo_fonctionnel = False

        #On augmente de 1 le nombre de périodes passées sans recevoir d'images de YOLO
        self.last_period_yolo_received += 1

#========================================================================================
#Fonction appelée lorsque l'on reçoit un nouveau message MAVROS indiquant l'état du robot
#========================================================================================
 
    def mavros_state_callback(self, msg: State):
        #On met à jour l'état d'armement du robot
        armed = msg.armed
        self.armed = armed
        self.get_logger().error(f"Etat de l'état de MAVROS : connecté {msg.connected},  armé : {msg.armed}, mode : {msg.mode}")
        #On met à jour le mode du robot 
        mode = msg.mode  # ex: "MANUAL", "GUIDED", "AUTO", etc.
        if mode in ["MANUAL", "GUIDED"]:
            self.mode = mode
        else:
            self.mode = None
        return None




