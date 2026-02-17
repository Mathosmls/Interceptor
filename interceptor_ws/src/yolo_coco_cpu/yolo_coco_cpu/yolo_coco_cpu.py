import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import time

class YoloCocoCpu(Node):
    def __init__(self):
        super().__init__('yolo_coco_cpu')

        # Paramètres
        self.declare_parameter('image_topic', '/interceptor_sim/interceptor/cam_rgb/image')
        self.declare_parameter('output_image_topic', '/camera/image_detected')
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('confidence_threshold', 0.15)
        self.declare_parameter('infer_size', 520)


        self.image_topic = self.get_parameter('image_topic').value
        self.output_image_topic = self.get_parameter('output_image_topic').value
        self.detection_topic = self.get_parameter('detection_topic').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.infer_size = self.get_parameter('infer_size').value

        # YOLOv8 COCO
        self.model = YOLO('yolov8n.pt')  # modèle léger
        self.bridge = CvBridge()

        # Sub / Pub
        self.sub_image = self.create_subscription(Image, self.image_topic, self.callback_raw, 10)
        self.pub_image = self.create_publisher(CompressedImage,  self.output_image_topic + "/compressed", 10)
        self.pub_detections = self.create_publisher(Detection2DArray, self.detection_topic, 10)
        self.sub_compressed = self.create_subscription(
            CompressedImage, '/hikrobot/image/compressed', self.callback_compressed, 10)

        self.get_logger().info("YoloCocoCpu initialized.")
        self.frame_count=0
        self.last_infer_time = 0
        self.min_period = 0.04  # 25 FPS max

    def callback_raw(self, msg: Image):
        now = time.time()
        if now - self.last_infer_time < self.min_period:
            return  # on ignore cette frame

        self.last_infer_time = now
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_image(cv_img, msg.header)


    def callback_compressed(self, msg: CompressedImage):
        now = time.time()
        if now - self.last_infer_time < self.min_period:
            return  # on ignore cette frame

        self.last_infer_time = now
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.process_image(cv_img, msg.header)


    def process_image(self, cv_img, header):

        # self.frame_count+=1

        orig_h, orig_w, _ = cv_img.shape

        # Masque bas image (sur image originale)
        mask_start = int(orig_h * 0.7)
        cv_img_masked = cv_img.copy()
        cv_img_masked[mask_start:orig_h, :] = 0

        # Resize AVANT YOLO
        infer_img = cv2.resize(cv_img_masked, (self.infer_size, self.infer_size))

        scale_x = orig_w / self.infer_size
        scale_y = orig_h / self.infer_size

        results = self.model(infer_img, verbose=False)[0]

        det_msg = Detection2DArray()
        det_msg.header = header

        best_detection = None
        best_score = 0.0
        best_box = None

        for box, cls_id, score in zip(results.boxes.xyxy, results.boxes.cls, results.boxes.conf):
            label = self.model.names[int(cls_id)]
            if label != 'boat':
                continue

            score = float(score)
            if score < self.conf_threshold:
                continue

            if score > best_score:
                best_score = score
                best_detection = label
                best_box = list(map(float, box))

        if best_detection is not None:
            # Reprojection bbox vers image ORIGINALE
            x1, y1, x2, y2 = best_box
            x1 *= scale_x
            x2 *= scale_x
            y1 *= scale_y
            y2 *= scale_y

            w = x2 - x1
            h = y2 - y1
            cx = x1 + w / 2
            cy = y1 + h / 2

            detection = Detection2D()
            detection.header = header
            detection.bbox.center.position.x = cx
            detection.bbox.center.position.y = cy
            detection.bbox.size_x = w
            detection.bbox.size_y = h

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = best_detection
            hyp.hypothesis.score = best_score
            detection.results.append(hyp)

            det_msg.detections.append(detection)
            # if self.frame_count % 2 == 0:
                # Dessin sur image ORIGINALE
            cv2.rectangle(cv_img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
            cv2.putText(cv_img, f'{best_detection} {best_score:.2f}',
                        (int(x1), int(y1) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)


        out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
        out_msg.header = header
        # if self.frame_count % 2 == 0:
        

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 60]  # 0–100
        preview = cv2.resize(cv_img, (960, 600))  # ou 640x400 si tu veux très léger
        result, encimg = cv2.imencode('.jpg', preview, encode_param)

        if result:
            comp_msg = CompressedImage()
            comp_msg.header = header
            comp_msg.format = "jpeg"
            comp_msg.data = encimg.tobytes()
            self.pub_image.publish(comp_msg)

        self.pub_detections.publish(det_msg)






def main(args=None):
    rclpy.init(args=args)
    node = YoloCocoCpu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
