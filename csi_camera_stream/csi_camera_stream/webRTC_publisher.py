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
        self.declare_parameter("mode", "video")  # TODO: choose between Options: "video", "still", "inference"
        self.declare_parameter("still_interval", 5.0)  # seconds for pictures
        self.declare_parameter("video_topic", "csi_video_stream") # video stream topic
        self.declare_parameter("still_topic", "csi_picture") # TODO: need to update these topics with actual name
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
        
        self.get_logger().info(f"WebRTC publisher started in '{self.mode}' mode")
        self.subcription = self.create_subscription(Image, self.video_topic, self.video_callback, 100)
        self.get_logger().info(f"Subscribed to video topic: {self.video_topic}")
        
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.start_webrtc_server())

    def video_callback(self, msg):
        self.get_logger().info("Received image message")  # Debug log

        self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        cv2.imshow("self frames", self.current_frame)
        cv2.waitKey(1)

        currentFrame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("frames without self", currentFrame)
        cv2.waitKey(1)

        if self.current_frame is not None:
            self.get_logger().info(f"Frame size: {self.current_frame.shape}")  # Should show (height, width, 3)
        else:
            self.get_logger().warn("Converted frame is None")


    # def still_callback(self, msg):
    #     try:
    #         self.last_still_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    #         self.last_still_time = time.time()
    #     except Exception as e:
    #         self.get_logger().error(f"Failed to convert still image: {e}")

    # def inference_callback(self, msg):
    #     try:
    #         self.last_inference_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    #         self.last_inference_time = time.time()
    #     except Exception as e:
    #         self.get_logger().error(f"Failed to convert inference frame: {e}")

    def get_current_frame(self):
        if self.mode == "video":
            frame = self.current_frame
        elif self.mode == "still":
            frame = self.last_still_frame
        elif self.mode == "inference":
            frame = self.last_inference_frame
        else:
            frame = None

        # if frame is None:
        #     self.get_logger().warning("No valid frame found, returning black frame.")
        # else:
        #     self.get_logger().info(f"Frame found: {frame.shape}, dtype={frame.dtype}")

        return frame

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
                self.get_logger().info(f"Received WebRTC message: {message}")  # Log incoming messages

                data = json.loads(message)
                if data.get("type") == "offer":
                    self.get_logger().info("WebRTC offer received, processing...")

                    # Close any existing peer connection
                    if self.pc:
                        await self.pc.close()
                    
                    self.pc = RTCPeerConnection()
                    
                    # Add ICE candidate handling
                    @self.pc.on("icecandidate")
                    async def on_ice_candidate(event):
                        if event.candidate:
                            self.get_logger().info(f"Sending ICE candidate: {event.candidate.candidate}")
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
                    
                    self.get_logger().info("Sending WebRTC answer")
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
