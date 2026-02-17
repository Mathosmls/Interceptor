import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float64

class PidBoat(Node):
    def __init__(self):
        super().__init__('pid_boat')

        # Paramètres
        self.declare_parameter('image_width', 960)  # largeur de l'image caméra
        self.declare_parameter('image_height', 600)  # largeur de l'image caméra
        self.declare_parameter('kp', 0.020)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.01)
        self.declare_parameter('max_center_error', 150.0)   # pixels
        self.declare_parameter('stop_box_ratio', 0.3)       # bbox width > image_width * ratio → stop
        self.declare_parameter('stop_box_ratio_height', 0.25)       # bbox width > image_width * ratio → stop
        self.declare_parameter('search_thrust', 5.)        # rotation quand on cherche
        self.declare_parameter('forward_thrust', 5)       # poussée avant quand on avance
        self.declare_parameter('lost_timeout', 1.5)  # secondes avant mode recherche
        
        self.image_height=self.get_parameter('image_height').value
        self.stop_box_ratio_height=self.get_parameter('stop_box_ratio_height').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        self.image_width = self.get_parameter('image_width').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.max_center_error = self.get_parameter('max_center_error').value
        self.stop_box_ratio = self.get_parameter('stop_box_ratio').value
        self.search_thrust = self.get_parameter('search_thrust').value
        self.forward_thrust = self.get_parameter('forward_thrust').value


        # PID state
        self.prev_error = 0.0
        self.integral = 0.0

        # Mémoire pour la perte de détection
        self.last_error = 0.0
        self.last_seen_time = self.get_clock().now()
        self.has_seen_target = False

        self.last_box_width = 0.0
        self.last_box_height = 0.0
        self.last_was_close = False
        self.center_tolerance = 80.0  # pixels pour considérer "centré quand proche"

        # Subscriptions
        self.sub_det = self.create_subscription(
            Detection2DArray,
            '/detections',  # topic de ton node YOLO
            self.callback_detection,
            10
        )

        # Publishers moteurs
        self.pub_left = self.create_publisher(Float64, '/interceptor_sim/interceptor/engine_left/cmd_thrust', 10)
        self.pub_right = self.create_publisher(Float64, '/interceptor_sim/interceptor/engine_right/cmd_thrust', 10)

        self.get_logger().info("PID initialized.")

    def callback_detection(self, msg: Detection2DArray):
        now = self.get_clock().now()
        image_center = self.image_width / 2

        # =====================================================
        # 1️⃣ SI DÉTECTION DISPONIBLE → mise à jour mémoire
        # =====================================================
        if msg.detections:
            det = msg.detections[0]
            cx = det.bbox.center.position.x
            box_width = det.bbox.size_x
            box_height = det.bbox.size_y

            error = image_center - cx

            self.last_error = error
            self.last_box_width = box_width
            self.last_box_height = box_height
            self.last_seen_time = now
            self.has_seen_target = True

            # Vérifie si la cible est proche ET centrée
            is_close = (
                box_width > self.image_width * self.stop_box_ratio or
                box_height > self.image_height * self.stop_box_ratio_height
            )
            is_centered = abs(error) < self.center_tolerance

            self.last_was_close = is_close and is_centered

        # =====================================================
        # 2️⃣ PAS DE DÉTECTION → utiliser mémoire
        # =====================================================
        else:
            if not self.has_seen_target:
                self.get_logger().info("Never seen target → searching")
                self.publish_motors(-self.search_thrust, self.search_thrust)
                return

            dt = (now - self.last_seen_time).nanoseconds * 1e-9

            # 🔴 Cas spécial : on était proche et centré → rester STOP
            if self.last_was_close and dt < self.lost_timeout:
                self.get_logger().info("Close target temporarily lost → staying stopped")
                self.publish_motors(0.0, 0.0)
                return

            if dt > self.lost_timeout:
                direction = -1.0 if self.last_error > 0 else 1.0
                self.get_logger().info("Target lost → searching in last known direction")
                self.publish_motors(direction * self.search_thrust,
                                    -direction * self.search_thrust)
                return


            # Sinon on continue avec dernière info connue
            error = self.last_error
            box_width = self.last_box_width
            box_height = self.last_box_height
            self.get_logger().info("Temporary loss → predicting motion")

        # =====================================================
        # 3️⃣ TROP PROCHE → STOP
        # =====================================================
        if (box_width > self.image_width * self.stop_box_ratio) or \
        (box_height > self.image_height * self.stop_box_ratio_height):
            self.get_logger().info("Target too close → stop")
            self.publish_motors(0.0, 0.0)
            return

        # =====================================================
        # 4️⃣ ERREUR TROP GRANDE → ROTATION SUR PLACE
        # =====================================================
        if abs(error) > self.max_center_error:
            direction = 1.0 if error > 0 else -1.0
            self.get_logger().info("Large error → turning in place")
            self.publish_motors(-direction * self.search_thrust/1.5,
                                direction * self.search_thrust/1.5)
            return

        # =====================================================
        # 5️⃣ PID NORMAL → AVANCE + CORRECTION
        # =====================================================
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error

        correction = self.kp * error + self.ki * self.integral + self.kd * derivative

        left_thrust = self.forward_thrust - correction
        right_thrust = self.forward_thrust + correction

        self.publish_motors(left_thrust, right_thrust)





    def publish_motors(self, left, right):
        self.pub_left.publish(Float64(data=left))
        self.pub_right.publish(Float64(data=right))


def main(args=None):
    rclpy.init(args=args)
    node = PidBoat()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
