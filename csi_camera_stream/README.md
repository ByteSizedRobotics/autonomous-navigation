## OpenCV with GStreamer Setup

### Installation and Build Process
- Built OpenCV from source with GStreamer support
- Followed comprehensive build guide: [OpenCV with GStreamer Build Tutorial](https://medium.com/@arfanmahmud47/build-opencv-4-from-source-with-gstreamer-ubuntu-zorin-peppermint-c2cff5393ef)

### Key Considerations
- Requires manually adding the `.so` built file to the corresponding virtual environment's `site-packages`
- Camera may require connection **before** Raspberry Pi boot

## Package Dependencies and Path Configuration

### Virtual Environment Setup
Packages are installed in a virtual environment. The Python scripts use custom path configurations:

```python
# Add virtual environment site-packages
sys.path.insert(0, "/home/adminbyte/venv/lib/python3.12/site-packages")

# Import required packages
from aiortc import RTCPeerConnection, VideoStreamTrack, RTCSessionDescription
import av
import asyncio
import json
import websockets
import time

# Add custom OpenCV build path
sys.path.insert(0, '/home/adminbyte/opencv/build/lib/python3')
import cv2
```

### Some Required Packages
- `aiortc`
- `av`
- `websockets`
- Custom OpenCV build with GStreamer support

## ROS2 Launch Steps
1. Build the package:
   ```bash
   colcon build --packages-select csi_camera_stream
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Launch the camera stream:
   ```bash
   ros2 launch csi_camera_stream csi_camera_stream.launch.py
   ```
