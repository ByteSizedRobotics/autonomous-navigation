import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from aiortc import RTCPeerConnection, VideoStreamTrack, RTCSessionDescription
import av
import asyncio
import json
import websockets
import time

class WebRTCPublisherNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("webrtc_publisher", anonymous=True)
        
        # Parameters
        self.mode = rospy.get_param("~mode", "video")  # Options: "video", "still", "inference"
        self.still_interval = rospy.get_param("~still_interval", 5.0)  # seconds
        self.video_topic = rospy.get_param("~video_topic", "raw_video_stream")
        self.still_topic = rospy.get_param("~still_topic", "csi_picture")
        self.inference_topic = rospy.get_param("~inference_topic", "detected_pothole_frames")
        
        # Initialize bridge and frame storage
        self.bridge = CvBridge()
        self.current_frame = None
        self.last_still_frame = None
        self.last_inference_frame = None
        self.last_still_time = 0
        self.last_inference_time = 0
        
        # WebRTC peer connection
        self.pc = None
        
        # Subscribe to topics based on mode
        self._setup_subscriptions()
            
        rospy.loginfo(f"WebRTC publisher started in '{self.mode}' mode")
        
        # Start WebRTC server
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.start_webrtc_server())

    def _setup_subscriptions(self):
        """Set up topic subscriptions based on current mode"""
        # Clear existing subscriptions
        for sub in getattr(self, '_subscriptions', []):
            sub.unregister()
        
        self._subscriptions = []
        
        # Subscribe to topics based on mode
        if self.mode in ["video", "all"]:
            sub = rospy.Subscriber(self.video_topic, Image, self.video_callback)
            self._subscriptions.append(sub)
            rospy.loginfo(f"Subscribed to video topic: {self.video_topic}")
            
        if self.mode in ["still", "all"]:
            sub = rospy.Subscriber(self.still_topic, Image, self.still_callback)
            self._subscriptions.append(sub)
            rospy.loginfo(f"Subscribed to still image topic: {self.still_topic}")
            
        if self.mode in ["inference", "all"]:
            sub = rospy.Subscriber(self.inference_topic, Image, self.inference_callback)
            self._subscriptions.append(sub)
            rospy.loginfo(f"Subscribed to inference topic: {self.inference_topic}")

    def video_callback(self, msg):
        """Callback function for receiving continuous video frames"""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Failed to convert video frame: {e}")

    def still_callback(self, msg):
        """Callback function for receiving still images"""
        try:
            self.last_still_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.last_still_time = time.time()
            rospy.logdebug("Received new still image")
        except Exception as e:
            rospy.logerr(f"Failed to convert still image: {e}")

    def inference_callback(self, msg):
        """Callback function for receiving inference frames"""
        try:
            self.last_inference_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.last_inference_time = time.time()
            rospy.logdebug("Received new inference frame")
        except Exception as e:
            rospy.logerr(f"Failed to convert inference frame: {e}")

    def get_current_frame(self):
        """Returns the appropriate frame based on the current mode"""
        if self.mode == "video":
            return self.current_frame
        elif self.mode == "still":
            return self.last_still_frame
        elif self.mode == "inference":
            return self.last_inference_frame
        elif self.mode == "all":
            # Priority: inference > still > video
            if self.last_inference_frame is not None and time.time() - self.last_inference_time < 1.0:
                return self.last_inference_frame
            elif self.last_still_frame is not None and time.time() - self.last_still_time < 1.0:
                return self.last_still_frame
            else:
                return self.current_frame
        else:
            return None

    class ROSVideoTrack(VideoStreamTrack):
        """Custom video track for WebRTC to fetch ROS images"""
        
        def __init__(self, publisher_node):
            super().__init__()
            self.publisher_node = publisher_node
            self.default_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            
        async def recv(self):
            frame_data = self.publisher_node.get_current_frame()
            
            if frame_data is not None:
                video_frame = av.VideoFrame.from_ndarray(frame_data, format="bgr24")
            else:
                video_frame = av.VideoFrame.from_ndarray(self.default_frame, format="bgr24")
                
            pts, time_base = await self.next_timestamp()
            video_frame.pts = pts
            video_frame.time_base = time_base
            return video_frame

    async def handle_offer(self, websocket, path):
        """Handles WebRTC offers and responds with answers"""
        async for message in websocket:
            data = json.loads(message)
            
            if data.get("type") == "offer":
                # Create a new peer connection for each offer
                self.pc = RTCPeerConnection()
                
                # Set the remote description
                offer = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
                await self.pc.setRemoteDescription(offer)
                
                # Add video track
                video_track = self.ROSVideoTrack(self)
                self.pc.addTrack(video_track)
                
                # Create answer
                answer = await self.pc.createAnswer()
                await self.pc.setLocalDescription(answer)
                
                # Send answer back to client
                response = json.dumps({"type": "answer", "sdp": self.pc.localDescription.sdp})
                await websocket.send(response)
                
                # Handle ICE candidates if needed
                @self.pc.on("icecandidate")
                async def on_ice_candidate(candidate):
                    if candidate:
                        await websocket.send(json.dumps({
                            "type": "candidate",
                            "candidate": candidate.candidate,
                            "sdpMid": candidate.sdpMid,
                            "sdpMLineIndex": candidate.sdpMLineIndex
                        }))
            
            elif data.get("type") == "candidate" and self.pc:
                candidate = data.get("candidate")
                sdpMid = data.get("sdpMid")
                sdpMLineIndex = data.get("sdpMLineIndex")
                await self.pc.addIceCandidate({
                    "candidate": candidate,
                    "sdpMid": sdpMid,
                    "sdpMLineIndex": sdpMLineIndex
                })
                
            elif data.get("type") == "config":
                # Handle configuration changes
                if "mode" in data:
                    new_mode = data["mode"]
                    if new_mode in ["video", "still", "inference", "all"]:
                        self.mode = new_mode
                        rospy.loginfo(f"Switched to {self.mode} mode")
                        # Update subscriptions
                        self._setup_subscriptions()
                    
                    response = json.dumps({"type": "config_response", "status": "ok", "mode": self.mode})
                    await websocket.send(response)

    async def start_webrtc_server(self):
        """Starts the WebRTC signaling server"""
        server = await websockets.serve(self.handle_offer, "0.0.0.0", 8765)
        rospy.loginfo("WebRTC signaling server started on port 8765")
        
        # Keep the server running
        await asyncio.Future()

if __name__ == "__main__":
    try:
        publisher = WebRTCPublisherNode()
    except rospy.ROSInterruptException:
        pass