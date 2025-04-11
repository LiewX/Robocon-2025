import cv2
from ultralytics import YOLO
import numpy as np
import math
import time

# Load the model
# yolo = YOLO("best35epoch.pt","segment")
YOLO_MODEL_PATH = "./weights/best.pt"
yolo = YOLO(YOLO_MODEL_PATH, "segment")

"""
NOTE: Press Q to quit the program and press P to pause the program

"""
# Load the video capture
videoCap = cv2.VideoCapture("./train_files/videos/testvid4.mp4")

# Known dimensions of the backboard
KNOWN_WIDTH = 1.83  # in meters
KNOWN_HEIGHT = 1.05  # in meters
FOCAL_LENGTH = 800  # Estimated focal length in pixels (adjust based on calibration)
KNOWN_DIAG = np.sqrt(np.square(KNOWN_WIDTH) + np.square(KNOWN_HEIGHT))
hope=None
TL_List=[] #top left median list
TR_List=[] #top right median list
BL_List=[] #bottom left median list
BR_List=[] #bottom right median list
MAX_LIST_SIZE=25
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

# just something to calc time taken for each frame to be processed currently on my laptop is around 0.08 seconds
# so 12.5 frames
prevtime=time.time()
while True:
    ret, frame = videoCap.read()
    if frame is None:
        break
    if not ret:
        continue
    
    #frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY) 
    #print(frame)
    results = yolo.track(frame, stream=True, verbose=False,persist=False)
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
                    # old code to calc the distance shouldnt use if got the new function to calc based on corners
                    bounding_box_width = abs(x1 - x2)
                    bounding_box_height = abs(y1 - y2)
                    bounding_box_diag = np.sqrt(np.square(bounding_box_width) + np.square(bounding_box_height))
                    distance = calculate_distance(KNOWN_DIAG, FOCAL_LENGTH, bounding_box_diag)
                    
                    # Extract and process the mask
                    
                    bruh=cv2.approxPolyDP(mask.xy[0],20,True)
                    hope=cv2.approxPolyN(bruh,4,None,0.3,True)
                    # getting the center of the bounding box
                    xy_coordinates = box.xywh[:, :2].cpu().numpy()
                    if len(TL_List) >= MAX_LIST_SIZE:
                        TL_List.pop(0)  # Remove the oldest item (first item) if the list is full
                        TR_List.pop(0)
                        BL_List.pop(0)
                        BR_List.pop(0)
                    # subtracting by the bounding box in order to reduce to large deviation that results from the median
                    TL_List.append(hope[0][0]-xy_coordinates)
                    TR_List.append(hope[0][1]-xy_coordinates)
                    BL_List.append(hope[0][2]-xy_coordinates)
                    BR_List.append(hope[0][3]-xy_coordinates)
                    # make into np vector to do median operation
                    TL_stacked = np.vstack(TL_List)
                    TR_stacked = np.vstack(TR_List)
                    BL_stacked = np.vstack(BL_List)
                    BR_stacked = np.vstack(BR_List)
                    # do median 
                    # NOTE: the TL, TR, BL and BR does not mean the top left and top right corner locations it changes sometimes
                    # like TL can sometimes contain the top left corner or bottom right corner or bottom left corner
                    # named it as is cuz i thought it would be consistent at first but turned out to not be
                    filtered_vector_TL = [np.median(TL_stacked[:, 0]),np.median(TL_stacked[:, 1])]
                    filtered_vector_TR = [np.median(TR_stacked[:, 0]),np.median(TR_stacked[:, 1])]
                    filtered_vector_BL = [np.median(BL_stacked[:, 0]),np.median(BL_stacked[:, 1])]
                    filtered_vector_BR = [np.median(BR_stacked[:, 0]),np.median(BR_stacked[:, 1])]
                    # highlighting the raw masking coming from the YOLO tracking in yellow
                    for point in mask.xy[0]:
                      cv2.circle(frame, (int(point[0]), int(point[1])), radius=1, color=(0, 255, 255), thickness=5)
                    # highlighting the corners detected by the filtering stuff in red
                    cv2.circle(frame, (int(filtered_vector_TL[0]+xy_coordinates[0][0]), int(filtered_vector_TL[1]+xy_coordinates[0][1])), radius=1, color=(0,0, 255), thickness=5)
                    cv2.circle(frame, (int(filtered_vector_TR[0]+xy_coordinates[0][0]), int(filtered_vector_TR[1]+xy_coordinates[0][1])), radius=1, color=(0,0, 255), thickness=5)
                    cv2.circle(frame, (int(filtered_vector_BL[0]+xy_coordinates[0][0]), int(filtered_vector_BL[1]+xy_coordinates[0][1])), radius=1, color=(0,0, 255), thickness=5)
                    cv2.circle(frame, (int(filtered_vector_BR[0]+xy_coordinates[0][0]), int(filtered_vector_BR[1]+xy_coordinates[0][1])), radius=1, color=(0,0, 255), thickness=5)
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
