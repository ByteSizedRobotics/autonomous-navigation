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

# WebRTC setup
pc = RTCPeerConnection()
bridge = CvBridge()
frame = None  # Global variable to hold the latest frame


class WebRTCBridgeNode(VideoStreamTrack):
    """Custom video track for WebRTC to fetch ROS images"""

    def __init__(self):
        super().__init__()

    async def recv(self):
        global frame
        if frame is not None:
            video_frame = av.VideoFrame.from_ndarray(frame, format="bgr24")
            return video_frame
        else:
            return av.VideoFrame.from_ndarray(np.zeros((480, 640, 3), dtype=np.uint8), format="bgr24")


def image_callback(msg):
    """Callback function for receiving camera images from ROS topic"""
    global frame
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr(f"Failed to convert image: {e}")


async def handle_offer(websocket):
    """Handles WebRTC offers and responds with answers"""
    global pc

    async for message in websocket:
        data = json.loads(message)

        if data["type"] == "offer":
            offer = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
            await pc.setRemoteDescription(offer)

            # Create video track
            video_track = ROSVideoTrack()
            pc.addTrack(video_track)

            # Create answer
            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)

            response = json.dumps({"type": "answer", "sdp": pc.localDescription.sdp})
            await websocket.send(response)


async def start_webrtc_server():
    """Starts the WebRTC signaling server"""
    async with websockets.serve(handle_offer, "0.0.0.0", 8765):
        await asyncio.Future()  # Run forever


if __name__ == "__main__":
    rospy.init_node("webrtc_publisher", anonymous=True)

    # Subscribe to the camera topic
    rospy.Subscriber("~image_raw", Image, image_callback)

    # Start WebRTC server
    loop = asyncio.get_event_loop()
    loop.run_until_complete(start_webrtc_server())
