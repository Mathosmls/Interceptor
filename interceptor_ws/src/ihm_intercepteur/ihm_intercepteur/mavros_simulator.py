#!/usr/bin/env python3

import random
import rclpy
from rclpy.node import Node

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode


class FakeMavrosNode(Node):

    def __init__(self):
        super().__init__('fake_mavros')

        # ==========================
        # Paramètres
        # ==========================
        self.p_accept = 0.8   # probabilité d'accepter l'armement
        self.armed = False
        self.mode = 'MANUAL'

        # ==========================
        # Publisher : état MAVROS
        # ==========================
        self.state_pub = self.create_publisher(
            State,
            '/mavros/state',
            10
        )

        # ==========================
        # Service : arming
        # ==========================
        self.arming_srv = self.create_service(
            CommandBool,
            '/mavros/cmd/arming',
            self.arming_callback
        )

        # ============================
        # Service : changement de mode
        # ============================
        self.mode_srv = self.create_service(
            SetMode,
            '/mavros/cmd/set_mode',
            self.mode_callback
        )

        # ==========================
        # Timer publication état
        # ==========================
        self.timer = self.create_timer(0.2, self.publish_state)  # 5 Hz

        self.get_logger().info("🧪 Fake MAVROS node started")

    # ==================================================
    # Callback service arming
    # ==================================================
    def arming_callback(self, request, response):
        desired_state = request.value  # True = ARM, False = DISARM
        self.get_logger().info(
            f"📨 Arming request received: {'ARM' if desired_state else 'DISARM'}"
        )

        # Tirage probabiliste
        if random.random() < self.p_accept:
            self.armed = desired_state
            response.success = True
            response.result = 0
            self.get_logger().info(
                f"✅ {'ARMED' if self.armed else 'DISARMED'} (accepted)"
            )
        else:
            response.success = False
            response.result = 1
            self.get_logger().warn(
                "❌ Arming command rejected (random failure)"
            )

        return response


    # ==================================================
    # Callback du service de changement de mode
    # ==================================================
    def mode_callback(self, request, response):
        desired_mode = request.custom_mode
        self.get_logger().info(
            f"Mode changing request received: {desired_mode}"
        )

        # Tirage probabiliste
        if random.random() < self.p_accept:
            self.mode = desired_mode
            response.mode_sent = True
            self.get_logger().info(
                f"✅ {desired_mode} (accepted)"
            )
        else:
            response.mode_sent = False
            self.get_logger().warn(
                "❌ Mode changing command rejected (random failure)"
            )

        return response

    # ==================================================
    # Publication de l'état MAVROS
    # ==================================================
    def publish_state(self):
        msg = State()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.connected = True
        msg.armed = self.armed
        msg.guided = True
        msg.manual_input = False
        msg.mode = self.mode
        msg.system_status = 4  # MAV_STATE_ACTIVE

        self.state_pub.publish(msg)


def main():
    rclpy.init()
    node = FakeMavrosNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
