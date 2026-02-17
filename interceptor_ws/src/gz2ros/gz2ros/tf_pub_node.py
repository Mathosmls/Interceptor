import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, TransformStamped
from tf2_ros import TransformBroadcaster
from functools import partial


class TfPubNode(Node):

    def __init__(self):
        super().__init__('tfPubNode')

        self.br = TransformBroadcaster(self)

        self.sub_interceptor = self.create_subscription(
            PoseArray,
            '/interceptor_sim/interceptor/pose',
            partial(self.callback, child_frame='base_link_interceptor'),
            10
        )

        self.sub_target = self.create_subscription(
            PoseArray,
            '/interceptor_sim/target/pose',
            partial(self.callback, child_frame='base_link_target'),
            10
        )
    
    def callback(self, msg: PoseArray, child_frame: str):
        if not msg.poses:
            return

        pose = msg.poses[0]

        t = TransformStamped()
        now = self.get_clock().now()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'map'        # parent frame
        t.child_frame_id = child_frame     # must match URDF root

        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z

        t.transform.rotation = pose.orientation

        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = TfPubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
