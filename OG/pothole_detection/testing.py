import sys
#sys.path.insert(0, '/home/adminbyte/opencv/build/lib/python3')
import cv2

cap = cv2.VideoCapture(8)

if not cap.isOpened():
    print("Error: Could not open CSI camera.")
    exit()

while True:
    success, frame = cap.read()
    if not success:
        print("Failed to capture frame")
        break

    cv2.imshow("CSI Camera", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
# cv2.destroyAllWindows()

