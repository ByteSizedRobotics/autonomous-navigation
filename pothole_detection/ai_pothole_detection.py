import os
import cv2
import torch
import argparse
import time
import threading

last_saved_time = 0
model = None

def load_model():
    global model
    model = torch.hub.load('ultralytics/yolov5', 'custom', path='best.pt')

# PERFORMS LIVE VIDEO INFERENCE
def live_camera_inference(pathSavedImages):
    # Define GStreamer pipeline for libcamera with IMX219 camera
    pipeline = (
        'libcamerasrc ! '
        'video/x-raw, width=640, height=480, framerate=30/1 ! '
        'videoconvert ! '
        'video/x-raw, format=BGR ! '
        'appsink'
    )
    
    # Create VideoCapture object with GStreamer pipeline
    camera = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    
    if not camera.isOpened():
        print("Error: Could not open camera with libcamera.")
        return
    
    print("Camera opened successfully with libcamera")

    while True:
        success, frame = camera.read()
        if not success:
            print("Failed to read frame from camera")
            break

        # Convert BGR to RGB for YOLOv5
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Run inference
        results = model(frame_rgb)

        # Render results
        results.render()
        rendered_frame = results.ims[0]
        
        # Convert back to BGR for OpenCV display
        display_frame = cv2.cvtColor(rendered_frame, cv2.COLOR_RGB2BGR)
        
        # Display the frame
        cv2.imshow('YOLOv5 Live', display_frame)

        # Save pictures in a separate thread
        threading.Thread(target=save_pictures, args=(frame, results, pathSavedImages)).start()

        if cv2.waitKey(1) & 0xFF == ord('q'):  # break the loop when press q
            break

    camera.release()
    cv2.destroyAllWindows()

# SAVES PICTURES EVERY 10 SECONDS IF OBJECT DETECTED WITH CONFIDENCE ABOVE THRESHOLD
def save_pictures(frame, results, pathSavedImages):
    CONFIDENCE_THRESHOLD = 0.5

    global last_saved_time
    current_time = time.time()

    if (current_time - last_saved_time >= 10):  # save images every 10 seconds if object detected with confidence above 50% 
        detections = results.xyxy[0].cpu().numpy()  # results.xyxy[0] contains all detected objects in format [x1, y1, x2, y2, confidence, class]

        # check if any object has confidence above threshold
        save_image = any(det[4] > CONFIDENCE_THRESHOLD for det in detections)

        if save_image:
            # Create directory if it doesn't exist
            os.makedirs(pathSavedImages, exist_ok=True)
            
            filename = os.path.join(pathSavedImages, f"capture_{int(current_time)}.jpg")
            cv2.imwrite(filename, frame)
            print(f"Image saved: {filename}")
        
        last_saved_time = current_time  # Update timestamp

# PERFORMS INFERENCE ON IMAGES IN A FOLDER
def image_inference(dataset_path, model):
    for filename in os.listdir(dataset_path):
        if filename.endswith(".jpg") or filename.endswith(".png"):
            image_path = os.path.join(dataset_path, filename)
            image = cv2.imread(image_path)
            
            if image is None:
                print(f"Error loading image: {image_path}")
                continue
                
            results = model(image)
        
            if results.xyxy[0].shape[0] > 0:
                results.show()
    cv2.destroyAllWindows()

# Main execution
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="YOLOv5 Object Detection with libcamera support")

    parser.add_argument("--mode", type=int, help="1 = image inference, 0 = video inference")
    parser.add_argument("--pathImg", type=str, help="Path to image folder, only needed if image inference was selected")
    parser.add_argument("--pathSaveImg", type=str, help="Location where you want to save the detected images from the video")
    args = parser.parse_args()

    # Load the model
    load_model()

    if args.mode == 1:
        if not args.pathImg:
            print("Error: --pathImg required for image inference mode")
            exit(1)
        image_inference(args.pathImg, model)
    else:
        if not args.pathSaveImg:
            print("Error: --pathSaveImg required for video inference mode")
            exit(1)
        live_camera_inference(args.pathSaveImg)