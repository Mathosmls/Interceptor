#!/usr/bin/env python3

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *


class ZoomableGraphicsView(QGraphicsView):
    def __init__(self, parent=None):
        super().__init__(parent)

        # =============== Attributs pour le zoom ===============
        self.zoom_factor = 1.0
        self.zoom_min = 0.2
        self.zoom_max = 20.0
        self.zoom_step = 1.15

        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.setDragMode(QGraphicsView.ScrollHandDrag)

        self.grabGesture(Qt.PinchGesture)

        # =============== Attributs pour le pan ===============
        #Booléen mis à True si il y a un panning en cours
        self._panning = False
        #Position du curseur de la souris au moment où on commence le pan
        self._pan_start = QPoint()

#====================================================================================================================
#Fonction appelée quand on détecte une tentative de zoom avec la molette (redéfinition d'une fonction de QMainWindow)
#====================================================================================================================

    def wheelEvent(self, event):
        #L'angle fait par la molette est nul, on ne fait rien
        if event.angleDelta().y() == 0:
            return
        #Le zoom/dézoom est plus ou moins grand selon l'angle que fait la molette
        #L'angle est positif : on zoome
        if event.angleDelta().y() > 0:
            self.apply_zoom(self.zoom_step)

        #L'angle est négatif : on dézoome
        else:
            self.apply_zoom(1 / self.zoom_step)

#=================================================================================
#Fonction détectant un évènement et appelant les fonctions qui leur sont associées
#=================================================================================

    def event(self, event):
        if event.type() == QEvent.Gesture:
            return self.gestureEvent(event)
        return super().event(event)


#===========================================================================================================================
#Fonction appelée quand on détecte une tentative de zoom avec le pavé tactile (redéfinition d'une fonction de QGraphicsView)
#===========================================================================================================================

    def gestureEvent(self, event):
        #renvoie un objet QPinchGesture si un pincement est détecté, renvoie None sinon
        pinch = event.gesture(Qt.PinchGesture)
        #Si un pincement est détecté, on appelle la fonction gérant les pincements de doigts sur le pavé tactile
        if pinch:
            self.handle_pinch(pinch)
            return True
        #Si aucun pincement n'est détecté, on ne fait rien
        return False

#=====================================================================================
#Fonction convertissant un pincement de doigts en facteur de zoom avant de l'appliquer
#=====================================================================================

    def handle_pinch(self, pinch: QPinchGesture):
        #Si les deux doigts viennent de toucher le pavé numérique (Qt), on stocke le facteur de zoom actuel
        if pinch.state() == Qt.GestureStarted:
            self.pinch_start_zoom = self.zoom_factor

        #Si les doigts bougent(Qt.GestureUpdated)
        elif pinch.state() == Qt.GestureUpdated:
            #Récupération du fcateur d'échelle (distance initiale entre les doigts/distance finale entre les doigts)
            scale_factor = pinch.scaleFactor()
            new_zoom = self.pinch_start_zoom * scale_factor

            #Si le zoom reste dans l'intervalle autorisé
            if self.zoom_min <= new_zoom <= self.zoom_max:
                factor = new_zoom / self.zoom_factor
                self.apply_zoom(factor)

#==================================
#Fonction appliquant le zoom/dézoom
#==================================

    def apply_zoom(self, factor, center_pos=None):
        #Le zoom doit être relatif au zoom que l'on avait à l'instant où le pincement a commencé et pas à celui que l'on avait au début de la simulation 
        #Ex: zoom de 2 au début du pincement, facteur d'échelle de 0,8, zoom désiré : 2*0.8 = 1.6
        new_zoom = self.zoom_factor * factor

        if not (self.zoom_min <= new_zoom <= self.zoom_max):
            return

        if center_pos is None:
            center_pos = self.mapToScene(self.mapFromGlobal(QCursor.pos()))

        # On stocke la position scène sous la souris avant le zoom
        old_pos = center_pos

        # On applique le zoom
        self.scale(factor, factor)
        self.zoom_factor = new_zoom

        # On regarde la position de la souris sur la scène après le zoom
        new_pos = self.mapToScene(self.mapFromGlobal(QCursor.pos()))

        # On translate la scène de la différence entre les deux positions
        delta = new_pos - old_pos
        self.translate(delta.x(), delta.y())

        #Mise à jour de la barre d'échelle
        self.parent().update_scale_bar()

        # Mise à jour de la carte GPS après le zoom
        if hasattr(self.parent(), 'update_map_view'):
            self.parent().update_map_view()
            
        return None


#==========================================================
#Fonction appelée lorsque l'on détecte un clic de la souris
#==========================================================

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._panning = True
            self._pan_start = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
        else:
            super().mousePressEvent(event)


#===============================================================
#Fonction appelée lorsque l'on détecte un mouvement de la souris
#===============================================================

    def mouseMoveEvent(self, event):
        #On vérifie qu'un pan est en cours 
        if self._panning:
            delta = event.pos() - self._pan_start
            self._pan_start = event.pos()

            self.horizontalScrollBar().setValue(
                self.horizontalScrollBar().value() - delta.x()
            )
            self.verticalScrollBar().setValue(
                self.verticalScrollBar().value() - delta.y()
            )

            event.accept()
        else:
            super().mouseMoveEvent(event)

#============================================================
#Fonction appelée lorsque l'on relâche un bouton de la souris
#============================================================

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and self._panning:
            self._panning = False
            self.setCursor(Qt.ArrowCursor)
            # Mise à jour de la carte GPS après le pan
            if hasattr(self.parent(), 'update_map_view'):
                self.parent().update_map_view()

            event.accept()
        else:
            super().mouseReleaseEvent(event)