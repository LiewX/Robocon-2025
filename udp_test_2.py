import socket
import numpy as np
import cv2
from ultralytics import YOLO
import math

# Define server details
HOST = '0.0.0.0'  # Listen on all available interfaces
PORT = 50007       # Same port as used by the client
# Load the model
yolo = YOLO('best.pt')
# Known dimensions of the backboard
KNOWN_WIDTH = 1.83  # in meters
KNOWN_HEIGHT = 1.05  # in meters
FOCAL_LENGTH = 800  # Estimated focal length in pixels (adjust based on calibration)
KNOWN_DIAG=np.sqrt(np.square(KNOWN_WIDTH)+np.square(KNOWN_HEIGHT))

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
    """
    Calculate the distance to the backboard.
    :param known_width: Real-world width of the backboard (meters)
    :param focal_length: Focal length of the camera (pixels)
    :param bounding_box_width: Width of the detected backboard in pixels
    :return: Distance to the backboard in meters
    """
    x=(known_diag * focal_length) / bounding_box_diag
    if (math.isnan(x)):
        return 0
    return x

# Create a UDP socket to listen for incoming data
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
    s.bind((HOST, PORT))  # Bind to the specified IP and port
    print(f"Server listening on {HOST}:{PORT}")

    while True:
        # Receive data from the client (Raspberry Pi)
        data, addr = s.recvfrom(65536)  # 64KB is a typical buffer size for large images

        # Decode the received image data
        image_array = np.frombuffer(data, dtype=np.uint8)
        frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
        frame=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
        results = yolo.track(frame, stream=True)

        for result in results:
            # Get the classes names
            classes_names = result.names

            # Iterate over each box
            for box in result.boxes:
                # Check if confidence is greater than 50%
                if box.conf[0] > 0.7:
                    # Get coordinates
                    [x1, y1, x2, y2] = box.xyxy[0]
                    # Convert to int
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                    # Get the class
                    cls = int(box.cls[0])

                    # Get the class name
                    class_name = classes_names[cls]

                    # Process only if the detected object is a backboard
                    if class_name.lower() == "backboard":
                        # Calculate width of the bounding box
                        bounding_box_width = abs(x1 - x2)
                        bounding_box_height=abs(y1-y2)
                        bounding_box_diag=np.sqrt(np.square(bounding_box_width)+np.square(bounding_box_height))

                        # Calculate distance
                        distance = calculate_distance(KNOWN_DIAG, FOCAL_LENGTH, bounding_box_diag)

                        # Extract the region of interest (ROI)
                        roi = frame[int(y1+200/distance):int(y2-100/distance), int(x1+300/distance):int(x2-300/distance)]
                        # Perform color detection in ROI
                        color_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

                        lower_white = np.array([150])
                        upper_white = np.array([255])   
                        mask=cv2.inRange(color_roi[:,:,1],lower_white,upper_white)
                        output=cv2.bitwise_and(color_roi[:,:,1],color_roi[:,:,1],mask=mask)
                        

                        # Perform edge detection on the ROI
                        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                        blur_roi=cv2.GaussianBlur(color_roi[:,:,1],(3,15),2)
                        
                        edges = cv2.Canny(blur_roi, 80, 300,1,L2gradient=1)
                        #contours, hierarchy=cv2.findContours(edges,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_NONE)
                        #if len(contours)==0:
                        #    pass
                        #else:
                        #    for j in contours:
                        #        x=np.concatenate((contours[0],j))
                        #    approx=cv2.approxPolyDP(x,20,1)
                        #    print(approx)
                        #    cv2.drawContours(edges, approx, -1, (255, 255, 255), 5)
                        #corners=cv2.goodFeaturesToTrack(edges,10,0.5,0.1)
                        
                        #if (corners is None):
                        #    pass
                        #else:        
                        #    for i in corners:
                        #        x,y = i.ravel()
                        #        cv2.circle(edges,(int(x),int(y)),3,255,-1)
                        
                        

                        

                        # Draw the rectangle
                        colour = getColours(cls)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)

                        # Put the class name, confidence, and distance on the image
                        cv2.putText(frame, f'{class_name} {box.conf[0]:.2f}', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, colour, 2)
                        cv2.putText(frame, f'Distance: {distance:.2f} m', (x1, y2), cv2.FONT_HERSHEY_SIMPLEX, 1, colour, 2)

                        # Display the edge-detected ROI
                        cv2.imshow("Edge Detection (ROI)", edges)

        # Display the received image
        if frame is not None:
            cv2.imshow('Received Image', frame)
        
        # If the user presses 'q', exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
