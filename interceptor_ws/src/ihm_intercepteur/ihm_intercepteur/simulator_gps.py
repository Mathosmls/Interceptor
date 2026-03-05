#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix


class TwoRobotsSimulator(Node):

    def __init__(self):
        super().__init__('two_robots_simulator')

        # =========================
        # Référence géographique
        # =========================
        self.lat_ref = 48.197717
        self.lon_ref = -3.016452
        self.alt_ref = 69.0

        # =========================
        # Trajectoire
        # =========================
        self.radius = 20.0   # mètres
        self.omega = 0.1     # rad/s
        self.dt = 0.05       # 20 Hz
        self.t = 0.0

        # =========================
        # Publishers MAVROS
        # =========================
        self.pub_gps_1 = self.create_publisher(
            NavSatFix,
            '/mavros/global_position/global',
            10
        )
        self.pub_hdg_1 = self.create_publisher(
            Float64,
            '/mavros/global_position/compass_hdg',
            10
        )

        self.pub_gps_2 = self.create_publisher(
            NavSatFix,
            '/mavros/global_position/global_cible',
            10
        )
        self.pub_hdg_2 = self.create_publisher(
            Float64,
            '/mavros/global_position/compass_hdg_cible',
            10
        )

        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info("Simulateur MAVROS (2 robots) démarré")

    # ======================================================
    # Conversion mètres → latitude / longitude
    # ======================================================
    def meters_to_latlon(self, north, east):
        dlat = north / 111111.0
        dlon = east / (111111.0 * math.cos(math.radians(self.lat_ref)))
        return dlat, dlon

    # =========================
    # Boucle principale
    # =========================
    def update(self):
        self.t += self.dt

        self.publish_robot(
            theta=self.omega * self.t,
            gps_pub=self.pub_gps_1,
            hdg_pub=self.pub_hdg_1
        )

        self.publish_robot(
            theta=self.omega * self.t,
            gps_pub=self.pub_gps_2,
            hdg_pub=self.pub_hdg_2
        )

    # =========================
    # Publication robot
    # =========================
    def publish_robot(self, theta, gps_pub, hdg_pub):

        # =========================
        # Position ENU → GPS
        # =========================
        north = 0#self.radius * math.cos(theta)
        east  = 0#self.radius * math.sin(theta)

        if gps_pub == self.pub_gps_2:
            east+=10

        dlat, dlon = self.meters_to_latlon(north, east)

        gps = NavSatFix()
        gps.header.stamp = self.get_clock().now().to_msg()
        gps.header.frame_id = "gps"

        gps.latitude = self.lat_ref + dlat
        gps.longitude = self.lon_ref + dlon
        gps.altitude = self.alt_ref

        gps_pub.publish(gps)

        # =========================
        # Heading (tangent trajectoire)
        # =========================
        yaw_rad = math.atan2(east, north)   # 0 = Nord
        yaw_deg = math.degrees(yaw_rad)
        yaw_deg = (yaw_deg + 360.0) % 360.0

        hdg = Float64()
        hdg.data = 0.0 #yaw_deg

        hdg_pub.publish(hdg)


def main():
    rclpy.init()
    node = TwoRobotsSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()