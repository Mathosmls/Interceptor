import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
import tf_transformations  # utile pour quaternions
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.time import Time

class ProcessLidar(Node):
    def __init__(self, z_min=0.0):
        super().__init__('process_lidar_node')
        self.z_min = z_min

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriber / Publisher
        self.sub = self.create_subscription(
            PointCloud2,
            '/interceptor_sim/interceptor/lidar_points',
            self.callback,
            10
        )
        self.pub = self.create_publisher(
            PointCloud2,
            '/interceptor_sim/interceptor/lidar_points_filtered',
            10
        )

    def callback(self, msg: PointCloud2):
        # Lire les points x,y,z
        points_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not points_list:
            return
        points = np.array([list(p) for p in points_list], dtype=np.float32)
        t_ros = Time(nanoseconds=msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)
        try:
            # Récupérer le tf du lidar vers map au moment du message
           trans = self.tf_buffer.lookup_transform(
                "map",
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(f"Impossible de récupérer le transform map <- {msg.header.frame_id}: {e}")
            return

        # Extraire rotation et translation
        t = np.array([
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z
        ], dtype=np.float32)
        q = trans.transform.rotation
        R = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]

        # Transformer tous les points dans map
        points_in_map = (R @ points.T).T + t  # shape [N,3]

        # Filtrer selon z dans map
        filtered = points_in_map[points_in_map[:, 2] > self.z_min]
        if filtered.size == 0:
            return

        # Créer un nouveau PointCloud2
        new_msg = pc2.create_cloud_xyz32(msg.header, filtered.tolist())
        self.pub.publish(new_msg)



def main():
    rclpy.init()
    node = ProcessLidar(z_min=0.5)  # <-- changer seuil ici
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
