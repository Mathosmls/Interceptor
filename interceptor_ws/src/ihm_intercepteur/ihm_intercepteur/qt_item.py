#!/usr/bin/env python3

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

import numpy as np


class ModularQtItem(object):
	"""
	Classe Item.
	Tous les membres de la classe possèdent un nom, une position, une orientation, etc...
	"""

	#Taille de la plus grande hauteur d'un item représenté par un triangle
	triangle_size = 18 

	def __init__(self, scene, name, color=(0,0,0), position=np.array([[None],[None],[None]]), orientation=0):
		"""
		- scene : scène sur laquelle on veut ajouter l'item
		- name : nom de l'item
		- color : tuple représentant la couleur RGB de l'item (noire par défaut) (ex : (R,G,B))
		- position de l'item : tuple contenant l'abscisse et l'ordonnée en m de l'item dans le repère du bateau (x,y)
		- orientation : float contenant l'orientation en degrés de l'item dans le repère du bateau 
		"""
		##Création de l'item 
		#Création d'un triangle isocèle (pointe avant, pointe droite, pointe gauche) 
		triangle = QPolygonF([QPointF(0, -ModularQtItem.triangle_size/2), QPointF(ModularQtItem.triangle_size / 4, ModularQtItem.triangle_size/2), QPointF(-ModularQtItem.triangle_size / 4, ModularQtItem.triangle_size/2)])
		self.item = QGraphicsPolygonItem(triangle)

		#Attribut stockant la scène à laquelle on veux ajouter l'item
		self.scene = scene
		#Ajout de l'item sur la scène
		self.scene.addItem(self.item)

		#Attribut stockant le nom de l'item (string)
		self.name = name

		#Mise à jour de sa couleur
		self.color = color
		self.item.setBrush(QBrush(QColor(color[0], color[1], color[2])))

		#Attribut contenant la position en m de l'item dans le repère du monde 
		self.position_NED = position

		#Attribut contenant l'orientation en degrés de l'item dans le repère du monde
		self.orientation_NED = orientation

		#Attribut contenant la position en m de l'item dans le repère de la scène / du bateau 
		self.position = position

		#Attribut contenant l'orientation en degrés de l'item dans le repère de la scène / du bateau 
		self.orientation = orientation

