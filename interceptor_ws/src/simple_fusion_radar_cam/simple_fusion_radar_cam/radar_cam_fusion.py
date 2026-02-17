import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from ars408_interfaces.msg import MsgObjects, MsgObject
import math
from visualization_msgs.msg import Marker
import time

class RadarCamFusion(Node):
    def __init__(self):
        super().__init__('radar_cam_fusion')

        # Paramètres caméra
        self.declare_parameter('camera_width', 1920)
        self.declare_parameter('camera_height', 1200)
        self.declare_parameter('camera_fov_h', 74.0)  # degrés
        self.declare_parameter('camera_fov_v', 57.0)

        # Topics
        self.declare_parameter('yolo_topic', '/detections')
        self.declare_parameter('radar_topic', '/radar/objects')
        self.declare_parameter('fusion_topic', '/radar/fusion/boat')

        self.width = self.get_parameter('camera_width').value
        self.height = self.get_parameter('camera_height').value
        self.fov_h = self.get_parameter('camera_fov_h').value

        self.yolo_topic = self.get_parameter('yolo_topic').value
        self.radar_topic = self.get_parameter('radar_topic').value
        self.fusion_topic = self.get_parameter('fusion_topic').value

        # Sub / Pub
        self.sub_yolo = self.create_subscription(Detection2DArray, self.yolo_topic, self.callback_yolo, 10)
        self.sub_radar = self.create_subscription(MsgObjects, self.radar_topic, self.callback_radar, 10)
        self.pub_fusion = self.create_publisher(MsgObject, self.fusion_topic, 10)
        self.pub_marker = self.create_publisher(Marker, '/radar/fusion/marker', 10)

        # Mémoire des derniers messages
        self.last_yolo = None
        self.last_radar = None
        self.max_angle_diff = math.radians(8)
        self.required_matches=0

        # Suivi basé sur le temps
        self.tracked_radar_id = None
        self.last_seen_time = 0.0
        self.lock_timeout = 0.6  # secondes avant de relâcher l'ID si plus vu
        self.max_allowed_angle_error = math.radians(10)  # tolérance angulaire avant relâchement
        self.max_error_duration = 0.5                     # secondes avant relâchement
        self.error_start_time = None

        self.get_logger().info("Radar-Camera Fusion Node started")

    # ----------------------
    # Callbacks
    # ----------------------
    def callback_yolo(self, msg: Detection2DArray):
        self.last_yolo = msg
        self.try_fuse()

    def callback_radar(self, msg: MsgObjects):
        self.last_radar = msg
        self.try_fuse()

    # ----------------------
    # Fusion logic
    # ----------------------
    def try_fuse(self):
        if self.last_radar is None:
            return

        now = time.time()
        closest_obj = None

        # --- Cas 1 : on suit déjà un ID radar verrouillé ---
        if self.tracked_radar_id is not None:
            tracked_obj = next((obj for obj in self.last_radar.objects
                                if obj.msg0x60b.id == self.tracked_radar_id), None)

            if tracked_obj is not None:
                self.last_seen_time = now

                # Vérifier divergence avec YOLO si disponible
                if self.last_yolo is not None:
                    best_det = None
                    best_score = 0.0
                    for det in self.last_yolo.detections:
                        if len(det.results) == 0:
                            continue
                        hyp = det.results[0]
                        if hyp.hypothesis.class_id != 'boat':
                            continue
                        if hyp.hypothesis.score > best_score:
                            best_score = hyp.hypothesis.score
                            best_det = det

                    if best_det is not None:
                        cx = best_det.bbox.center.position.x
                        focal_pixels = (self.width / 2) / math.tan(math.radians(self.fov_h / 2))
                        angle_yolo_rad = math.atan2(cx - self.width/2, focal_pixels)

                        dx = tracked_obj.msg0x60b.dist_long
                        dy = tracked_obj.msg0x60b.dist_lat
                        angle_tracked = -math.atan2(dy, dx)
                        angle_error = abs((angle_yolo_rad - angle_tracked + math.pi) % (2*math.pi) - math.pi)

                        if angle_error > self.max_allowed_angle_error:
                            if self.error_start_time is None:
                                self.error_start_time = now
                            elif now - self.error_start_time > self.max_error_duration:
                                self.get_logger().warn(f"Radar target {self.tracked_radar_id} diverged from YOLO, releasing lock")
                                self.tracked_radar_id = None
                                self.match_count = 0
                                self.error_start_time = None
                        else:
                            self.error_start_time = None

                # Publier fusion
                self.pub_fusion.publish(tracked_obj)
                self.pub_marker_u(tracked_obj)
                self.get_logger().info(f"Object tracked by radar ID {self.tracked_radar_id}")
                return
            elif now - self.last_seen_time > self.lock_timeout:
                self.get_logger().warn(f"Lost tracked radar target {self.tracked_radar_id}, releasing lock")
                self.tracked_radar_id = None
                self.match_count = 0
                self.error_start_time = None

        # --- Cas 2 : YOLO détecte le bateau et on n'a pas de radar verrouillé ---
        if self.last_yolo is not None and self.tracked_radar_id is None:
            best_det = None
            best_score = 0.0
            for det in self.last_yolo.detections:
                if len(det.results) == 0:
                    continue
                hyp = det.results[0]
                if hyp.hypothesis.class_id != 'boat':
                    continue
                if hyp.hypothesis.score > best_score:
                    best_score = hyp.hypothesis.score
                    best_det = det

            if best_det is not None:
                # Calcul angle YOLO
                cx = best_det.bbox.center.position.x
                focal_pixels = (self.width / 2) / math.tan(math.radians(self.fov_h / 2))
                angle_yolo_rad = math.atan2(cx - self.width/2, focal_pixels)

                # Chercher tous les objets radar dans l'angle
                candidates = []
                for obj in self.last_radar.objects:
                    dx = obj.msg0x60b.dist_long
                    dy = obj.msg0x60b.dist_lat
                    obj_angle = -math.atan2(dy, dx)
                    angle_diff = abs((obj_angle - angle_yolo_rad + math.pi) % (2*math.pi) - math.pi)

                    if angle_diff < self.max_angle_diff:
                        distance = math.hypot(dx, dy)
                        candidates.append((distance, obj))

                if candidates:
                    candidates.sort(key=lambda x: x[0])
                    closest_obj = candidates[0][1]

                    # --- Gestion du verrouillage par matches ---
                    if getattr(self, 'last_candidate_id', None) == closest_obj.msg0x60b.id:
                        self.match_count += 1
                    else:
                        self.last_candidate_id = closest_obj.msg0x60b.id
                        self.match_count = 1

                    if self.match_count >= self.required_matches:
                        self.tracked_radar_id = closest_obj.msg0x60b.id
                        self.last_seen_time = now
                        self.get_logger().info(f"Radar target LOCKED: {self.tracked_radar_id}")

                    # Publier fusion et marker (même si pas encore locké)
                    self.pub_fusion.publish(closest_obj)
                    self.pub_marker_u(closest_obj)
                    return
                else:
                    self.get_logger().info("No radar object in YOLO angle sector")
                    self.match_count = 0  # reset si pas de correspondance

        # Si on n’a ni radar verrouillé ni correspondance YOLO
        self.get_logger().info("No object to fuse or track")


    # ----------------------
    # Marker
    # ----------------------
    def pub_marker_u(self,obj):
        marker = Marker()
        marker.header.frame_id = "map"  # repère du radar
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = obj.msg0x60b.dist_long
        marker.pose.position.y = obj.msg0x60b.dist_lat
        marker.pose.position.z = 0.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Taille
        marker.scale.x = 0.5
        marker.scale.y = obj.msg0x60d.length
        marker.scale.z = obj.msg0x60d.width

        # Couleur
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        self.pub_marker.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = RadarCamFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()