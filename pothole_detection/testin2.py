import cv2
import torch
import subprocess
import time
import threading
from pathlib import Path

# Global variables
last_saved_time = 0
model = None

# Load YOLO model
def load_model():
    global model
    # model_path = str(Path("best.pt"))
    # model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=True)
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)


# Live video inference with libcamera-vid
def live_camera_inference():
    global model

    # Start libcamera-vid stream
    cmd = [
        "libcamera-vid",
        "-t", "0",               # Run indefinitely
        "--width", "640",
        "--height", "480",
        "--framerate", "30",
        "-o", "-",               # Output to stdout
        "--codec", "mjpeg"        # MJPEG format
    ]

    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=10**8)

    cap = cv2.VideoCapture("pipe:0", cv2.CAP_FFMPEG)

    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            break

        # Run YOLO inference
        results = model(frame)
        results.render()
        frame = results.ims[0]

        threading.Thread(target=save_pictures, args=(frame, results)).start()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    process.terminate()
    cv2.destroyAllWindows()

# Save detected images
def save_pictures(frame, results):
    CONFIDENCE_THRESHOLD = 0.5
    global last_saved_time
    current_time = time.time()

    if (current_time - last_saved_time >= 5):  
        detections = results.xyxy[0].cpu().numpy()
        save_image = any(det[4] > CONFIDENCE_THRESHOLD for det in detections)

        if save_image:
            cv2.imwrite(f"/app/saved_images/capture_{int(current_time)}.jpg", frame)

        last_saved_time = current_time

# Start live inference
load_model()
threading.Thread(target=live_camera_inference, args=()).start()
