import rclpy
from rclpy.node import Node
import os
import pathlib
from pathlib import Path
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import threading
import time
from queue import Queue

from picamera2 import Picamera2
from libcamera import Transform

sys.path.insert(0, '/home/adminbyte/opencv/build/lib/python3')
import cv2
import time

sys.path.insert(0, '/home/adminbyte/venv/lib/python3.12/site-packages')
import numpy as np
import torch

class CSIVideoNode(Node):
    def __init__(self):
        super().__init__('csi_video_node')

        # Parameters
        self.declare_parameter('width', 1640)
        self.declare_parameter('height', 1232)
        self.declare_parameter('fps', 30)
        self.declare_parameter('camera_frame_id', 'camera')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('debug_mode', True)
        self.declare_parameter('inference_every_n_frames', 30)

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.inference_every_n_frames = self.get_parameter('inference_every_n_frames').value

        # Publishers
        self.image_pub = self.create_publisher(Image, 'csi_video_stream', 1)

        # Bridge
        self.bridge = CvBridge()

        # Camera info
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = self.camera_frame_id
        self.camera_info_msg.width = self.width
        self.camera_info_msg.height = self.height

        fx = self.width / 2.0
        fy = self.height / 2.0
        cx = self.width / 2.0
        cy = self.height / 2.0
        self.camera_info_msg.distortion_model = "plumb_bob"
        self.camera_info_msg.d = [0.0] * 5
        self.camera_info_msg.k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.camera_info_msg.r = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        self.camera_info_msg.p = [fx, 0, cx, 0, fy, cy, 0, 0, 0, 1, 0]

        # Debug
        self.debug_dir = None
        if self.debug_mode:
            self.debug_dir = str(Path(__file__).parent / "debug_images")
            os.makedirs(self.debug_dir, exist_ok=True)
            self.saved_image_counter = 0
            self.get_logger().info("DEBUG MODE ENABLED - detection frames will be saved")

        # Inference state
        self.latest_detections = []
        self.new_detection_available = False
        self.infer_lock = threading.Lock()
        self.frame_q = Queue(maxsize=1)
        self.stop_flag = False

        # Load YOLO
        self.model = None
        self.model_path = str(Path("/home/adminbyte/ros2_ws/src/autonomous-navigation/csi_camera_stream/csi_camera_stream/best.pt"))
        self.load_model()
        self.start_inference_thread()

        # Start Picamera2
        self.start_camera()

    def load_model(self):
        try:
            self.get_logger().info(f"Loading YOLOv5 model from {self.model_path}")
            self.model = torch.hub.load(
                'ultralytics/yolov5',
                'custom',
                path=self.model_path,
                force_reload=False
            )
            self.model.conf = float(self.confidence_threshold)
            self.model.iou = 0.45
            self.model.max_det = 50
            torch.set_num_threads(os.cpu_count())
            self.get_logger().info("YOLOv5 model loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLOv5 model: {e}")
            raise

    def start_inference_thread(self):
        def worker():
            while not self.stop_flag:
                try:
                    frame = self.frame_q.get(timeout=0.1)  # lores frame (BGR)
                except:
                    continue
                try:
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    results = self.model(rgb, size=640)
                    dets = results.xyxy[0].cpu().numpy()
                    with self.infer_lock:
                        self.latest_detections = dets
                        self.new_detection_available = True
                except Exception as e:
                    self.get_logger().error(f"Inference error: {e}")
        t = threading.Thread(target=worker, daemon=True)
        t.start()

    def start_camera(self):
        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"size": (self.width, self.height), "format": "BGR888"},
            lores={"size": (640, 480), "format": "BGR888"},
            transform=Transform()
        )
        self.picam2.configure(config)
        self.picam2.start()

        self.get_logger().info("Picamera2 started successfully")

        frame_count = 0
        try:
            while rclpy.ok():
                lores = self.picam2.capture_array("lores")
                frame = self.picam2.capture_array("main")

                frame_count += 1
                now = self.get_clock().now().to_msg()

                # Send lores frame for inference
                if frame_count % self.inference_every_n_frames == 0:
                    if self.frame_q.full():
                        try:
                            self.frame_q.get_nowait()
                        except:
                            pass
                    try:
                        self.frame_q.put_nowait(lores)
                    except:
                        pass

                # Draw detections
                detection_found = False
                new_detection = False
                with self.infer_lock:
                    dets = self.latest_detections.copy()
                    new_detection = self.new_detection_available
                    self.new_detection_available = False

                scale_x = frame.shape[1] / lores.shape[1]
                scale_y = frame.shape[0] / lores.shape[0]

                for x1, y1, x2, y2, conf, cls in dets:
                    if conf >= self.confidence_threshold:
                        detection_found = True
                        label = f"{self.model.names[int(cls)]}:{conf:.2f}"
                        x1, y1, x2, y2 = int(x1 * scale_x), int(y1 * scale_y), int(x2 * scale_x), int(y2 * scale_y)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(frame, label, (x1, y1 - 8),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Save debug images
                if self.debug_mode and detection_found and new_detection and self.debug_dir:
                    filename = f"detection_{time.strftime('%Y%m%d_%H%M%S')}_{self.saved_image_counter:04d}.jpg"
                    cv2.imwrite(os.path.join(self.debug_dir, filename), frame)
                    self.saved_image_counter += 1

                # Publish ROS Image
                msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                msg.header.stamp = now
                msg.header.frame_id = self.camera_frame_id
                self.image_pub.publish(msg)

        finally:
            self.stop_flag = True
            self.picam2.stop()
            self.get_logger().info("Camera stopped and resources cleaned up")


def main(args=None):
    rclpy.init(args=args)
    node = CSIVideoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
