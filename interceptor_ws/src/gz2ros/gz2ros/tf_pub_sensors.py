import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TransformStamped, Pose
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf_transformations import quaternion_from_matrix, translation_from_matrix
import numpy as np
import tf2_ros
import tf2_geometry_msgs

class TfPubSensors(Node):
    def __init__(self):
        super().__init__('tf_pub_sensors')

        self.br = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Données par robot
        self.robots = {
            'interceptor': {
                'gps': None,
                'imu': None,
                'frame_gps': 'gps_interceptor',
                'frame_imu': 'imu_interceptor',
                'frame_base': 'base_link_interceptor',
                'ref_lat': 48.199227,
                'ref_lon': -3.014601,
                'ref_alt': 0.
            },
            'target': {
                'gps': None,
                'imu': None,
                'frame_gps': 'gps_target',
                'frame_imu': 'imu_target',
                'frame_base': 'base_link_target',
                'ref_lat': 48.199227,
                'ref_lon': -3.014601,
                'ref_alt': 0.
            }
        }

        # Subscriptions
        self.create_subscription(NavSatFix, '/interceptor_sim/interceptor/gps',
                                 lambda msg: self.gps_cb(msg, 'interceptor'), 10)
        self.create_subscription(Imu, '/interceptor_sim/interceptor/imu',
                                 lambda msg: self.imu_cb(msg, 'interceptor'), 50)
        self.create_subscription(NavSatFix, '/interceptor_sim/target/gps',
                                 lambda msg: self.gps_cb(msg, 'target'), 10)
        self.create_subscription(Imu, '/interceptor_sim/target/imu',
                                 lambda msg: self.imu_cb(msg, 'target'), 50)

        # Timer 20 Hz
        self.create_timer(0.05, self.publish_tf)

    def gps_cb(self, msg: NavSatFix, robot_name: str):
        robot = self.robots[robot_name]

        # Convertir latitude/longitude/altitude en x, y, z (ENU) par rapport à ref
        R_earth = 6378137.0
        dlat = np.radians(msg.latitude - robot['ref_lat'])
        dlon = np.radians(msg.longitude - robot['ref_lon'])
        x = R_earth * dlon * np.cos(np.radians(robot['ref_lat']))
        y = R_earth * dlat
        z = msg.altitude - robot['ref_alt']

        # Stocker la position dans le frame du capteur
        robot['gps'] = Pose()
        robot['gps'].position.x = x
        robot['gps'].position.y = y
        robot['gps'].position.z = z

    def imu_cb(self, msg: Imu, robot_name: str):
        robot = self.robots[robot_name]
        robot['imu'] = msg.orientation  # orientation dans le frame imu

    def publish_tf(self):
        now = self.get_clock().now().to_msg()
        for name, robot in self.robots.items():
            if robot['gps'] is None or robot['imu'] is None:
                continue

            try:
                # On récupère les TF statiques des capteurs vers base_link
                t_gps_to_base = self.tf_buffer.lookup_transform(
                     robot['frame_gps'],robot['frame_base'], rclpy.time.Time())
                t_imu_to_base = self.tf_buffer.lookup_transform(
                     robot['frame_imu'],robot['frame_base'], rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                continue

            # Transformer la position GPS dans base_link
            gps_in_base = tf2_geometry_msgs.do_transform_pose(
                Pose(position=robot['gps'].position, orientation=robot['imu']),
                t_gps_to_base
            )

            # Transformer l'orientation IMU dans base_link
            imu_pose = Pose()
            imu_pose.orientation = robot['imu']
            imu_in_base = tf2_geometry_msgs.do_transform_pose(imu_pose, t_imu_to_base)

            # Créer le TF global du base_link dans map
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "map"
            t.child_frame_id = robot['frame_base']

            # Position GPS transformée
            t.transform.translation.x = gps_in_base.position.x
            t.transform.translation.y = gps_in_base.position.y
            t.transform.translation.z = gps_in_base.position.z

            # Orientation IMU transformée
            t.transform.rotation = imu_in_base.orientation

            self.br.sendTransform(t)



def main():
    rclpy.init()
    node = TfPubSensors()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
