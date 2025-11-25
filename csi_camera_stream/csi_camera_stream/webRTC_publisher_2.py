import rclpy
import sys
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
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

class WebRTCPublisher2Node(Node):
    def __init__(self):
        super().__init__("webrtc_publisher_2")
        
        # Parameters
        self.declare_parameter("video_topic", "csi_video_stream_2/compressed")  # Compressed video stream topic
        self.declare_parameter("webrtc_port", 8767)  # WebRTC signaling server port

        self.bridge = CvBridge()
        self.webrtc_port = self.get_parameter("webrtc_port").value
        self.current_frame = None
        
        self.peer_connections = {}  # Track multiple WebRTC peer connections
        self.connection_id = 0  # Counter for unique connection IDs

        video_topic = self.get_parameter("video_topic").value
        self.get_logger().info(f"WebRTC publisher 2 started, subscribing to: {video_topic}")
        self.get_logger().info(f"WebRTC signaling server will start on port: {self.webrtc_port}")
        
        # Subscribe to compressed image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            video_topic,
            self.video_callback,
            10
        )
        
        # Start WebRTC in a separate thread
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.run_webrtc_server, daemon=True).start()
    
    def run_webrtc_server(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.start_webrtc_server())
    
    def video_callback(self, msg):
        # Decode compressed JPEG image
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.current_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
    def get_current_frame(self):
        frame = self.current_frame
        return frame

    class ROSVideoTrack(VideoStreamTrack):
        def __init__(self, publisher_node):
            super().__init__()
            self.publisher_node = publisher_node
            self.default_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
            self.last_frame_time = 0
            self.frame_interval = 1.0 / 30.0  # 30 FPS target
        
        async def recv(self):
            # Rate limit frame delivery to prevent buffering
            import time
            current_time = time.time()
            time_since_last = current_time - self.last_frame_time
            if time_since_last < self.frame_interval:
                await asyncio.sleep(self.frame_interval - time_since_last)
            self.last_frame_time = time.time()
            
            frame_data = self.publisher_node.get_current_frame()
            video_frame = av.VideoFrame.from_ndarray(frame_data if frame_data is not None else self.default_frame, format="bgr24")
            pts, time_base = await self.next_timestamp()
            video_frame.pts = pts
            video_frame.time_base = time_base
            return video_frame

    async def handle_offer(self, websocket, path=None):
        conn_id = None
        pc = None
        try:
            async for message in websocket:
                self.get_logger().info(f"Received WebRTC message: {message}")  # Log incoming messages

                data = json.loads(message)
                if data.get("type") == "offer":
                    # Create a new unique connection ID
                    conn_id = self.connection_id
                    self.connection_id += 1
                    
                    self.get_logger().info(f"WebRTC offer received for connection {conn_id}, processing...")
                    
                    # Create a new peer connection for this client
                    pc = RTCPeerConnection()
                    self.peer_connections[conn_id] = pc
                    
                    self.get_logger().info(f"Active connections: {len(self.peer_connections)}")
                    
                    # Add connection state change handler
                    @pc.on("connectionstatechange")
                    async def on_connectionstatechange():
                        self.get_logger().info(f"Connection {conn_id} state: {pc.connectionState}")
                        if pc.connectionState in ["failed", "closed"]:
                            if conn_id in self.peer_connections:
                                del self.peer_connections[conn_id]
                                self.get_logger().info(f"Removed connection {conn_id}. Active connections: {len(self.peer_connections)}")
                    
                    # Add ICE candidate handling
                    @pc.on("icecandidate")
                    async def on_ice_candidate(event):
                        if event.candidate:
                            self.get_logger().info(f"Sending ICE candidate for connection {conn_id}")
                            await websocket.send(json.dumps({
                                "type": "ice-candidate", 
                                "candidate": event.candidate.candidate
                            }))
                    
                    offer = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
                    await pc.setRemoteDescription(offer)
                    
                    # Create a new video track instance for this connection
                    video_track = self.ROSVideoTrack(self)
                    pc.addTrack(video_track)
                    
                    answer = await pc.createAnswer()
                    await pc.setLocalDescription(answer)
                    
                    self.get_logger().info(f"Sending WebRTC answer for connection {conn_id}")
                    await websocket.send(json.dumps({
                        "type": "answer", 
                        "sdp": pc.localDescription.sdp
                    }))
        except Exception as e:
            self.get_logger().error(f"WebRTC signaling error: {e}")
        finally:
            # Clean up this specific connection
            if conn_id is not None and conn_id in self.peer_connections:
                pc = self.peer_connections[conn_id]
                await pc.close()
                del self.peer_connections[conn_id]
                self.get_logger().info(f"Cleaned up connection {conn_id}. Active connections: {len(self.peer_connections)}")

    async def start_webrtc_server(self):
        server = await websockets.serve(self.handle_offer, "0.0.0.0", self.webrtc_port)
        self.get_logger().info(f"WebRTC signaling server started on port {self.webrtc_port}")
        await asyncio.Future()


def main(args=None):
    rclpy.init(args=args)
    node = WebRTCPublisher2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
