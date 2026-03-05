#!/usr/bin/env python3
"""
Gestionnaire de carte GPS utilisant Leaflet.js via QWebEngineView
Cette version offre une meilleure qualité de rendu que les tuiles natives PyQt
"""

from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QUrl, pyqtSlot, QObject
import math


class LeafletMapManager(QObject):
    """
    Gestionnaire de carte utilisant Leaflet.js pour un rendu haute qualité
    """
    
    def __init__(self, central_lat, central_lon, scale):
        """
        :param central_lat: Latitude centrale de référence (degrés)
        :param central_lon: Longitude centrale de référence (degrés)
        :param scale: Échelle en pixels/mètre
        """
        super().__init__()
        
        self.central_lat = central_lat
        self.central_lon = central_lon
        self.scale = scale
        
        # Rayon de la Terre en mètres
        self.EARTH_RADIUS = 6378137.0
        
        # Créer la vue web
        self.web_view = QWebEngineView()
        #On met la vue web en arrière plan
        self.web_view.lower()
        
        # Charger la carte
        self.load_map()
        
    def load_map(self):
        """Charge la carte Leaflet dans la WebView"""
        html = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="utf-8"/>
            <style>
                html, body, #map {{
                    margin: 0;
                    padding: 0;
                    width: 100%;
                    height: 100%;
                    overflow: hidden;
                }}
            </style>
            <link rel="stylesheet"
                  href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
            <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
        </head>
        <body>
            <div id="map"></div>
            <script>
                var map = L.map('map', {{
                    zoomControl: false,
                    attributionControl: false,
                    inertia: false,
                    zoomAnimation: false,
                    fadeAnimation: false,
                    markerZoomAnimation: false,
                    dragging: false,
                    touchZoom: false,
                    scrollWheelZoom: false,
                    doubleClickZoom: false,
                    boxZoom: false,
                    keyboard: false
                }}).setView(
                    [{self.central_lat}, {self.central_lon}], 17
                );
                
                L.tileLayer(
                    'https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png',
                    {{ 
                        maxZoom: 19,
                        minZoom: 0,
                        tileSize: 256,
                        updateWhenIdle: false,
                        updateWhenZooming: false,
                        keepBuffer: 4
                    }}
                ).addTo(map);
                
                // Fonction pour définir le zoom
                window.setZoom = function(z) {{
                    map.setZoom(z, {{ animate: false }});
                }};
                
                // Fonction pour définir le centre
                window.setCenter = function(lat, lon) {{
                    map.setView([lat, lon], map.getZoom(), {{ animate: false }});
                }};
                
                // Fonction pour pan
                window.panTo = function(lat, lon) {{
                    map.panTo([lat, lon], {{ animate: false }});
                }};
            </script>
        </body>
        </html>
        """
        self.web_view.setHtml(html)
    
    def lat_lon_to_meters(self, lat, lon):
        """
        Convertit des coordonnées lat/lon en mètres relatifs au point central
        
        :return: (x_meters, y_meters) relatif au centre
        """
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        central_lat_rad = math.radians(self.central_lat)
        central_lon_rad = math.radians(self.central_lon)
        
        # Projection Web Mercator
        x = self.EARTH_RADIUS * (lon_rad - central_lon_rad)
        
        y = self.EARTH_RADIUS * math.log(
            math.tan(math.pi / 4 + lat_rad / 2) / 
            math.tan(math.pi / 4 + central_lat_rad / 2)
        )
        
        return x, y
    
    def meters_to_lat_lon(self, x_meters, y_meters):
        """
        Convertit des coordonnées en mètres (relatif au centre) en lat/lon
        
        :return: (lat, lon) en degrés
        """
        central_lat_rad = math.radians(self.central_lat)
        central_lon_rad = math.radians(self.central_lon)
        
        lon_rad = central_lon_rad + x_meters / self.EARTH_RADIUS
        
        lat_rad = 2 * math.atan(
            math.exp(y_meters / self.EARTH_RADIUS) * 
            math.tan(math.pi / 4 + central_lat_rad / 2)
        ) - math.pi / 2
        
        return math.degrees(lat_rad), math.degrees(lon_rad)
    
    def get_optimal_zoom_level(self, view_zoom_factor):
        """
        Détermine le niveau de zoom Leaflet optimal
        
        :param view_zoom_factor: Facteur de zoom actuel de la vue
        :return: Niveau de zoom Leaflet (0-19)
        """
        base_zoom = 17
        #Le x6 est présent pour que cela donne les vraies dimensions à l'écran
        zoom_adjustment = math.log2(view_zoom_factor*7.4)
        optimal_zoom = int(base_zoom + zoom_adjustment)
        return max(0, min(19, optimal_zoom))
    
    def update_map(self, view_rect, view_zoom_factor, view_center_scene):
        """
        Met à jour la position et le zoom de la carte Leaflet
        
        :param view_rect: QRectF de la zone visible dans la scène
        :param view_zoom_factor: Facteur de zoom actuel
        :param view_center_scene: QPointF du centre de la vue en coordonnées scène
        """
        # Calculer le niveau de zoom
        zoom_level = self.get_optimal_zoom_level(view_zoom_factor)
        
        # Convertir le centre de la vue en lat/lon
        center_x_m = view_center_scene.x() / self.scale
        center_y_m = -view_center_scene.y() / self.scale
        center_lat, center_lon = self.meters_to_lat_lon(center_x_m, center_y_m)
        
        # Mettre à jour Leaflet
        js_code = f"setZoom({zoom_level}); setCenter({center_lat}, {center_lon});"
        self.web_view.page().runJavaScript(js_code)
    
    def resize_view(self, width, height):
        """Redimensionne la vue web"""
        self.web_view.setGeometry(0, 0, width, height)
    
    def show(self):
        """Affiche la carte"""
        self.web_view.show()
        self.web_view.lower()  # Mettre en arrière-plan
    
    def hide(self):
        """Cache la carte"""
        self.web_view.hide()
