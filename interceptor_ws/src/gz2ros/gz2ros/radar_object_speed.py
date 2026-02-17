import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ars408_interfaces.msg import MsgObjects


class RadarObjectSpeed(Node):

    def __init__(self):
        super().__init__('radar_object_speed')

        # Paramètre : ID de l'objet à suivre
        self.declare_parameter("target_id", 18)
        self.target_id = self.get_parameter("target_id").value

        # Subscriber
        self.sub = self.create_subscription(
            MsgObjects,
            '/radar/objects',
            self.objects_callback,
            10
        )

        # Publisher vitesse
        self.pub = self.create_publisher(Twist, '/radar/tracked_object_twist', 10)

        self.get_logger().info(f"Tracking radar object ID = {self.target_id}")

    def objects_callback(self, msg: MsgObjects):
        found = False

        for obj in msg.objects:
            # Sécurité : vérifier que msg0x60b existe
            if not hasattr(obj, "msg0x60b"):
                continue

            if obj.msg0x60b.id == self.target_id:
                twist = Twist()
                twist.linear.x = obj.msg0x60b.vrel_long
                twist.linear.y = obj.msg0x60b.vrel_lat
                twist.linear.z = 0.0

                # Pas d'info d'angle ici
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = 0.0

                self.pub.publish(twist)
                found = True
                break

        if not found:
            self.get_logger().debug(f"Object ID {self.target_id} not found in this frame")


def main(args=None):
    rclpy.init(args=args)
    node = RadarObjectSpeed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
