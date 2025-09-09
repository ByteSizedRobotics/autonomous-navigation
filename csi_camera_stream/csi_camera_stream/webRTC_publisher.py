import rclpy
import sys
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import queue
import time

sys.path.insert(0, "/home/adminbyte/venv/lib/python3.12/site-packages")
from aiortc import RTCPeerConnection, VideoStreamTrack, RTCSessionDescription
import av
import asyncio
import json
import websockets

sys.path.insert(0, '/home/adminbyte/opencv/build/lib/python3')
import cv2

class WebRTCPublisherNode(Node):
    def __init__(self):
        super().__init__("webrtc_publisher")
        
        # Parameters
        self.declare_parameter("video_topic", "csi_video_stream")
        self.declare_parameter("stream_width", 3280)   # 3280x2464 for streaming
        self.declare_parameter("stream_height", 2464)   # 3280x2464 for streaming
        self.declare_parameter("stream_fps", 30)       # Lower FPS for streaming
        self.declare_parameter("quality", 80)          # JPEG quality 0-100

        self.video_topic = self.get_parameter("video_topic").value
        self.stream_width = self.get_parameter("stream_width").value
        self.stream_height = self.get_parameter("stream_height").value
        self.stream_fps = self.get_parameter("stream_fps").value
        self.quality = self.get_parameter("quality").value

        self.bridge = CvBridge()
        
        # Frame management with thread-safe queue
        self.frame_queue = queue.Queue(maxsize=2)  # Small queue to prevent lag
        self.frame_count = 0
        self.last_frame_time = 0
        self.frame_interval = 1.0 / self.stream_fps
        
        self.pc = None
        
        self.get_logger().info(f"WebRTC publisher started - Stream: {self.stream_width}x{self.stream_height}@{self.stream_fps}fps")
        
        # Subscribe to camera stream
        self.subscription = self.create_subscription(
            Image, 
            self.video_topic, 
            self.video_callback, 
            1  # Small queue size
        )
        self.get_logger().info(f"Subscribed to video topic: {self.video_topic}")
        
        # Start WebRTC in separate thread
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.run_webrtc_server, daemon=True).start()
    
    def run_webrtc_server(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.start_webrtc_server())
    
    def video_callback(self, msg):
        current_time = time.time()
        
        # Frame rate limiting - only process frames at desired FPS
        if current_time - self.last_frame_time < self.frame_interval:
            return
        
        try:
            # Convert ROS image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Downscale frame for streaming performance (1280x720 -> 1080x720)
            if frame.shape[1] != self.stream_width or frame.shape[0] != self.stream_height:
                frame = cv2.resize(frame, (self.stream_width, self.stream_height), interpolation=cv2.INTER_LINEAR)
            
            # Add frame to queue (non-blocking)
            try:
                # Remove old frame if queue is full
                if self.frame_queue.full():
                    try:
                        self.frame_queue.get_nowait()
                    except queue.Empty:
                        pass
                
                self.frame_queue.put_nowait(frame)
                self.last_frame_time = current_time
                self.frame_count += 1
                
            except queue.Full:
                pass  # Skip frame if still full
                
        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")

    def get_current_frame(self):
        try:
            # Get most recent frame (non-blocking)
            frame = self.frame_queue.get_nowait()
            return frame
        except queue.Empty:
            # Return black frame if no frame available
            return np.zeros((self.stream_height, self.stream_width, 3), dtype=np.uint8)

    class ROSVideoTrack(VideoStreamTrack):
        def __init__(self, publisher_node):
            super().__init__()
            self.publisher_node = publisher_node
            self.frame_rate = publisher_node.stream_fps
            self.frame_time = 1.0 / self.frame_rate
            self.last_frame_time = 0
            
        async def recv(self):
            current_time = time.time()
            
            # Frame rate limiting for WebRTC
            if current_time - self.last_frame_time < self.frame_time:
                await asyncio.sleep(self.frame_time - (current_time - self.last_frame_time))
            
            frame_data = self.publisher_node.get_current_frame()
            
            # Create video frame
            video_frame = av.VideoFrame.from_ndarray(frame_data, format="bgr24")
            
            # Set timing
            pts, time_base = await self.next_timestamp()
            video_frame.pts = pts
            video_frame.time_base = time_base
            
            self.last_frame_time = time.time()
            return video_frame

    async def handle_offer(self, websocket, path=None):
        try:
            async for message in websocket:
                data = json.loads(message)
                
                if data.get("type") == "offer":
                    self.get_logger().info("Processing WebRTC offer...")

                    # Close existing connection
                    if self.pc:
                        await self.pc.close()
                    
                    # Create new peer connection with optimized settings
                    self.pc = RTCPeerConnection()
                    
                    # ICE candidate handling
                    @self.pc.on("icecandidate")
                    async def on_ice_candidate(event):
                        if event.candidate:
                            await websocket.send(json.dumps({
                                "type": "ice-candidate", 
                                "candidate": event.candidate.candidate
                            }))
                    
                    @self.pc.on("connectionstatechange")
                    async def on_connection_state_change():
                        if self.pc:  # Check if pc still exists
                            self.get_logger().info(f"WebRTC connection state: {self.pc.connectionState}")
                    
                    # Set remote description
                    offer = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
                    await self.pc.setRemoteDescription(offer)
                    
                    # Add optimized video track
                    video_track = self.ROSVideoTrack(self)
                    self.pc.addTrack(video_track)
                    
                    # Create and send answer
                    answer = await self.pc.createAnswer()
                    await self.pc.setLocalDescription(answer)
                    
                    await websocket.send(json.dumps({
                        "type": "answer", 
                        "sdp": self.pc.localDescription.sdp
                    }))
                    
                    self.get_logger().info("WebRTC answer sent")
                    
                elif data.get("type") == "ice-candidate":
                    # Handle incoming ICE candidates
                    if self.pc and data.get("candidate"):
                        await self.pc.addIceCandidate(data["candidate"])
                        
        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info("WebSocket connection closed")
        except Exception as e:
            self.get_logger().error(f"WebRTC signaling error: {e}")
        finally:
            if self.pc:
                await self.pc.close()
                self.pc = None

    async def start_webrtc_server(self):
        self.get_logger().info("Starting WebRTC signaling server on port 8765")
        
        server = await websockets.serve(
            self.handle_offer, 
            "0.0.0.0", 
            8765,
            ping_interval=20,
            ping_timeout=10,
            close_timeout=10
        )
        
        self.get_logger().info("WebRTC signaling server ready")
        await server.wait_closed()


def main(args=None):
    rclpy.init(args=args)
    node = WebRTCPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.pc:
            # Cleanup WebRTC connection
            try:
                asyncio.run(node.pc.close())
            except:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
