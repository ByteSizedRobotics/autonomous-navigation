import rclpy
import sys
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading

sys.path.insert(0, "/home/adminbyte/venv/lib/python3.12/site-packages")
from aiortc import RTCPeerConnection, VideoStreamTrack, RTCSessionDescription
import av
import asyncio
import json
import websockets
import time

sys.path.insert(0, '/home/adminbyte/opencv/build/lib/python3')
import cv2

class USBWebRTCPublisherNode(Node):
    def __init__(self):
        super().__init__("usb_webrtc_publisher")
        
        # Parameters
        self.declare_parameter("video_topic", "usb_video_stream")  # USB video stream topic

        self.bridge = CvBridge()
        self.current_frame = None
        
        self.pc = None  # WebRTC peer connection

        self.get_logger().info(f"USB WebRTC publisher started")
        self.subscription = self.create_subscription(Image, 'usb_video_stream', self.video_callback, 10)
        
        # Start WebRTC in a separate thread
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.run_webrtc_server, daemon=True).start()
    
    def run_webrtc_server(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.start_webrtc_server())
    
    def video_callback(self, msg):
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
    def get_current_frame(self):
        frame = self.current_frame
        return frame

    class ROSVideoTrack(VideoStreamTrack):
        def __init__(self, publisher_node):
            super().__init__()
            self.publisher_node = publisher_node
            self.default_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        
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
        server = await websockets.serve(self.handle_offer, "0.0.0.0", 8766)
        self.get_logger().info("USB WebRTC signaling server started on port 8766")
        await asyncio.Future()


def main(args=None):
    rclpy.init(args=args)
    node = USBWebRTCPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
