import numpy as np

#Convertit des données (latitude, longitude, altitude) dans le repère ECEF
def geodetic_to_ecef(lat, lon, alt):
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


#Convertit les coodronnées du repère ECEF dans le repère NED
def ecef_to_ned(ecef, ecef_ref, lat_ref, lon_ref):
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



def pose_and_velocity_callback():
    lat_ref = 48.4199312 #degrés
    long_ref = -3.015653 #degrés
    alt_ref = 69 #m
    latitude, longitude, altitude = 48.4199312, -3.015653, 70
    ecef_coordinates = geodetic_to_ecef(latitude, longitude, altitude)
    ecef_ref = geodetic_to_ecef(lat_ref, long_ref, alt_ref)
    ned_coordinates = ecef_to_ned(ecef_coordinates, ecef_ref, lat_ref, long_ref)
    #Mise à jour de la position de l'intercepteur dans le repère NED
    print("Coordonnées NED",ned_coordinates)

pose_and_velocity_callback()


