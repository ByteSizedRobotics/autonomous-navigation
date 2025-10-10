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

class WebRTCPublisherNode(Node):
    def __init__(self):
        super().__init__("webrtc_publisher")
        
        # Parameters
        # self.declare_parameter("still_interval", 5.0)  # seconds for pictures
        self.declare_parameter("video_topic", "csi_video_stream") # video stream topic
        # self.declare_parameter("still_topic", "csi_picture") # TODO: need to update these topics with actual name
        # self.declare_parameter("inference_topic", "detected_pothole_frames")

        #self.still_interval = self.get_parameter("still_interval").value
        #self.still_topic = self.get_parameter("still_topic").value
        #self.inference_topic = self.get_parameter("inference_topic").value

        self.bridge = CvBridge()
        self.current_frame = None
        
        self.peer_connections = {}  # Track multiple WebRTC peer connections
        self.connection_id = 0  # Counter for unique connection IDs

        self.get_logger().info(f"WebRTC publisher started")
        self.subscription = self.create_subscription(Image, 'csi_video_stream', self.video_callback, 10)
        
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
            self.default_frame = np.zeros((1232, 1640, 3), dtype=np.uint8)
        
        async def recv(self):
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