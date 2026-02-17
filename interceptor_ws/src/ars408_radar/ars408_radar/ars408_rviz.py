import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray
from ars408_interfaces.msg import MsgObjects


class RadarObjectsRViz(Node):
    def __init__(self):
        super().__init__('radar_objects_rviz')

        self.sub = self.create_subscription(
            MsgObjects,
            'objects',
            self.objects_callback,
            10
        )

        self.pub = self.create_publisher(
            MarkerArray,
            'radar/objects_markers',
            10
        )

    def objects_callback(self, msg: MsgObjects):
        marker_array = MarkerArray()

        for i, obj in enumerate(msg.objects):
            if not obj.msg0x60b or not obj.msg0x60d or not obj.msg0x60c:
                continue

            b = obj.msg0x60b
            c = obj.msg0x60c
            d = obj.msg0x60d

            # =====================
            # Cube (radar object)
            # =====================
            marker = Marker()
            marker.header = msg.header
            marker.ns = "radar_objects"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = b.dist_long
            marker.pose.position.y = b.dist_lat
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.25
            marker.scale.y = d.length
            marker.scale.z = d.width

            # --- Couleur via RCS ---
            rcs = b.rcs
            rcs_min = -25.0
            rcs_max = 25.0

            rcs_norm = (rcs - rcs_min) / (rcs_max - rcs_min)
            rcs_norm = max(0.0, min(1.0, rcs_norm))

            # to incrase color contrast
            gamma = 1.5  
            rcs_norm = rcs_norm ** gamma

            marker.color.r = rcs_norm
            marker.color.g = 0.0
            marker.color.b = 1.0 - rcs_norm
            marker.color.a = 0.8

            marker.lifetime.nanosec = 200_000_000

            marker_array.markers.append(marker)

            # =====================
            # Text (other info)
            # =====================
            text_marker = Marker()
            text_marker.header = msg.header
            text_marker.ns = "radar_objects_text"
            text_marker.id = 1000 + i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = b.dist_long
            text_marker.pose.position.y = b.dist_lat
            text_marker.pose.position.z = 1.5

            text_marker.scale.z = 0.4
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            text_marker.text = (
                f"ID {b.id}\n"
                f"Prob {c.prob_of_exist}/7\n"
                f"RCS {b.rcs:.1f} dBsm"
            )

            text_marker.lifetime.nanosec = 200_000_000

            marker_array.markers.append(text_marker)

        self.pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = RadarObjectsRViz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
