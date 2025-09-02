import cv2 # THIS IS THE OPENCV BUILT FROM SOURCE WITH GSTREAMER SUPPORT
import subprocess as sp
import shlex
import os

# Define the FIFO (named pipe)
fifo_path = "/tmp/camera_pipe"

# Ensure the FIFO does not exist before creating
if os.path.exists(fifo_path):
    os.remove(fifo_path)
os.mkfifo(fifo_path)

# Start libcamera-vid process
cmd = f"libcamera-vid -t 0 --width 640 --height 640 --framerate 30 --codec mjpeg --inline -o {fifo_path}" # this opens new cam window (add --nopreview if want no window)
process = sp.Popen(shlex.split(cmd))

# OpenCV Capture from the named pipe
cap = cv2.VideoCapture(fifo_path, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Failed to open camera pipe")
    process.terminate()
    process.wait()
    os.remove(fifo_path)
    exit(1)

print("Camera opened successfully using libcamera")

# Display video stream
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to read frame")
        break

    cv2.imshow("CSI Camera Stream", frame) # this also opens cam window for received frames

    # Break on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
process.terminate()
process.wait()
os.remove(fifo_path)

print("Camera stopped")