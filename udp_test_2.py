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
        data, addr = s.recvfrom(32768)  # 32KB is a typical buffer size for large images
        # can change the above value to tune the latency

        # Decode the received image data
        image_array = np.frombuffer(data, dtype=np.uint8)
        frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
        frame=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
        
        # start yolo tracking with the frames provided from the RPI

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
                        continue


        # Display the received image
        if frame is not None:
            cv2.imshow('Received Image', frame)
        
        # If the user presses 'q', exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
