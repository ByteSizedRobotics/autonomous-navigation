import rclpy
import sys
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

sys.path.insert(0, "/home/adminbyte/venv/lib/python3.12/site-packages")
from aiortc import RTCPeerConnection, VideoStreamTrack, RTCSessionDescription
import av
import asyncio
import json
import websockets
import time

sys.path.insert(0, '/home/adminbyte/opencv/build/lib/python3')
import cv2

class WebRTCPublisherNode(Node):
    def __init__(self):
        super().__init__("webrtc_publisher")
        
        # Parameters
        self.declare_parameter("mode", "video")  # Options: "video", "still", "inference"
        self.declare_parameter("still_interval", 5.0)  # seconds
        self.declare_parameter("video_topic", "raw_video_stream")
        self.declare_parameter("still_topic", "csi_picture")
        self.declare_parameter("inference_topic", "detected_pothole_frames")

        self.mode = self.get_parameter("mode").value
        self.still_interval = self.get_parameter("still_interval").value
        self.video_topic = self.get_parameter("video_topic").value
        self.still_topic = self.get_parameter("still_topic").value
        self.inference_topic = self.get_parameter("inference_topic").value

        self.bridge = CvBridge()
        self.current_frame = None
        self.last_still_frame = None
        self.last_inference_frame = None
        self.last_still_time = 0
        self.last_inference_time = 0
        
        self.pc = None  # WebRTC peer connection
        
        self._setup_subscriptions()
        self.get_logger().info(f"WebRTC publisher started in '{self.mode}' mode")
        
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.start_webrtc_server())

    def _setup_subscriptions(self):
        if hasattr(self, '_subscriptions'):
            for sub in self._subscriptions:
                self.destroy_subscription(sub)
        self._subscriptions = []
        
        if self.mode in ["video", "all"]:
            sub = self.create_subscription(Image, self.video_topic, self.video_callback, 10)
            self._subscriptions.append(sub)
            self.get_logger().info(f"Subscribed to video topic: {self.video_topic}")
        
        if self.mode in ["still", "all"]:
            sub = self.create_subscription(Image, self.still_topic, self.still_callback, 10)
            self._subscriptions.append(sub)
            self.get_logger().info(f"Subscribed to still image topic: {self.still_topic}")
        
        if self.mode in ["inference", "all"]:
            sub = self.create_subscription(Image, self.inference_topic, self.inference_callback, 10)
            self._subscriptions.append(sub)
            self.get_logger().info(f"Subscribed to inference topic: {self.inference_topic}")

    def video_callback(self, msg):
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert video frame: {e}")

    def still_callback(self, msg):
        try:
            self.last_still_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.last_still_time = time.time()
        except Exception as e:
            self.get_logger().error(f"Failed to convert still image: {e}")

    def inference_callback(self, msg):
        try:
            self.last_inference_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.last_inference_time = time.time()
        except Exception as e:
            self.get_logger().error(f"Failed to convert inference frame: {e}")

    def get_current_frame(self):
        if self.mode == "video":
            return self.current_frame
        elif self.mode == "still":
            return self.last_still_frame
        elif self.mode == "inference":
            return self.last_inference_frame
        elif self.mode == "all":
            if self.last_inference_frame and time.time() - self.last_inference_time < 1.0:
                return self.last_inference_frame
            elif self.last_still_frame and time.time() - self.last_still_time < 1.0:
                return self.last_still_frame
            return self.current_frame
        return None

    class ROSVideoTrack(VideoStreamTrack):
        def __init__(self, publisher_node):
            super().__init__()
            self.publisher_node = publisher_node
            self.default_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        
        async def recv(self):
            frame_data = self.publisher_node.get_current_frame()
            video_frame = av.VideoFrame.from_ndarray(frame_data if frame_data is not None else self.default_frame, format="bgr24")
            pts, time_base = await self.next_timestamp()
            video_frame.pts = pts
            video_frame.time_base = time_base
            return video_frame

        async def handle_offer(self, websocket, path=None):
            try:
                async for message in websocket:
                    data = json.loads(message)
                    if data.get("type") == "offer":
                        # Close any existing peer connection
                        if self.pc:
                            await self.pc.close()
                        
                        self.pc = RTCPeerConnection()
                        
                        # Add ICE candidate handling
                        @self.pc.on("icecandidate")
                        async def on_ice_candidate(event):
                            if event.candidate:
                                await websocket.send(json.dumps({
                                    "type": "ice-candidate", 
                                    "candidate": event.candidate.candidate
                                }))
                        
                        offer = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
                        await self.pc.setRemoteDescription(offer)
                        
                        video_track = self.ROSVideoTrack(self)
                        self.pc.addTrack(video_track)
                        
                        answer = await self.pc.createAnswer()
                        await self.pc.setLocalDescription(answer)
                        
                        await websocket.send(json.dumps({
                            "type": "answer", 
                            "sdp": self.pc.localDescription.sdp
                        }))
            except Exception as e:
                self.get_logger().error(f"WebRTC signaling error: {e}")
            finally:
                if self.pc:
                    await self.pc.close()

    async def start_webrtc_server(self):
        server = await websockets.serve(self.handle_offer, "0.0.0.0", 8765)
        self.get_logger().info("WebRTC signaling server started on port 8765")
        await asyncio.Future()


def main(args=None):
    rclpy.init(args=args)
    node = WebRTCPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
