import cv2
from ultralytics import YOLO
import numpy as np
import math
import time

# Load the model
yolo = YOLO("best.pt")


# Load the video capture
videoCap = cv2.VideoCapture("testvid4.mp4")

# Known dimensions of the backboard
KNOWN_WIDTH = 1.83  # in meters
KNOWN_HEIGHT = 1.05  # in meters
FOCAL_LENGTH = 800  # Estimated focal length in pixels (adjust based on calibration)
KNOWN_DIAG = np.sqrt(np.square(KNOWN_WIDTH) + np.square(KNOWN_HEIGHT))
hope=None
# Function to get class colors
def getColours(cls_num):
    base_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
    color_index = cls_num % len(base_colors)
    increments = [(1, -2, 1), (-2, 1, -1), (1, -1, 2)]
    color = [
        base_colors[color_index][i] + increments[color_index][i] * (cls_num // len(base_colors)) % 256
        for i in range(3)
    ]
    return tuple(color)

# Function to calculate distance
def calculate_distance(known_diag, focal_length, bounding_box_diag):
    x = (known_diag * focal_length) / bounding_box_diag
    return 0 if math.isnan(x) else x
prevtime=time.time()
while True:
    ret, frame = videoCap.read()
    if frame is None:
        break
    if not ret:
        continue
    #frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY) 
    #print(frame)
    results = yolo.track(frame, stream=True, verbose=False, persist=True)
    for result in results:
        classes_names = result.names
        if result.masks is None:  # Skip iteration if no masks are detected
            continue
        for box, mask in zip(result.boxes, result.masks):
            if box.conf[0] > 0.7:
                [x1, y1, x2, y2] = map(int, box.xyxy[0])
                cls = int(box.cls[0])
                class_name = classes_names[cls]

                if class_name.lower() == "backboard":
                    bounding_box_width = abs(x1 - x2)
                    bounding_box_height = abs(y1 - y2)
                    bounding_box_diag = np.sqrt(np.square(bounding_box_width) + np.square(bounding_box_height))
                    distance = calculate_distance(KNOWN_DIAG, FOCAL_LENGTH, bounding_box_diag)
                    
                    # Extract and process the mask
                    bruh=cv2.approxPolyDP(mask.xy[0],20,True)
                    hope=cv2.approxPolyN(bruh,4,hope,0.3,True)
                    print(hope)
                    for point in mask.xy[0]:
                        cv2.circle(frame, (int(point[0]), int(point[1])), radius=1, color=(0, 255, 255), thickness=5)
                    for point2 in hope[0]:
                        cv2.circle(frame, (int(point2[0]), int(point2[1])), radius=1, color=(0,0, 255), thickness=5)
                    # Draw bounding box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), getColours(cls), 2)
                    
                    # Display information
                    cv2.putText(frame, f'{class_name} {box.conf[0]:.2f}', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, getColours(cls), 2)
                    cv2.putText(frame, f'Distance: {distance:.2f} m', (x1, y2), cv2.FONT_HERSHEY_SIMPLEX, 1, getColours(cls), 2)
    
    cv2.imshow('frame', frame)

    
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
    if key == ord('p'):
        cv2.waitKey(-1)  # Pause until a key is pressed
    curtime=time.time()-prevtime
    prevtime=time.time()
    print(curtime)

videoCap.release()
cv2.destroyAllWindows()
