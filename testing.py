import cv2
import numpy as np
import math
from ultralytics import YOLO

# Load the model (adjust path to your best.pt file)
yolo = YOLO('best.pt')

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

def calculate_distance(known_diag, focal_length, bounding_box_diag):
    """
    Calculate the distance to the backboard.
    :param known_diag: Real-world diagonal of the backboard (meters)
    :param focal_length: Focal length of the camera (pixels)
    :param bounding_box_diag: Diagonal of the bounding box (pixels)
    :return: Distance to the backboard in meters
    """
    x = (known_diag * focal_length) / bounding_box_diag
    if math.isnan(x):
        return 0
    return x

def detect_and_highlight_backboard(image_path):
    # Load the image
    image = cv2.imread(image_path)
    if image is None:
        print("Error loading image!")
        return
    orig = image.copy()

    # Known dimensions of the backboard
    KNOWN_WIDTH = 1.83  # in meters
    KNOWN_HEIGHT = 1.05  # in meters
    FOCAL_LENGTH = 400*1.2147  # Estimated focal length in pixels (adjust based on calibration)
    KNOWN_DIAG = np.sqrt(np.square(KNOWN_WIDTH) + np.square(KNOWN_HEIGHT))
    horizontal_dist=7.125
    vertical_dist=2.43+KNOWN_HEIGHT/2
    Real_dist=math.sqrt(np.square(horizontal_dist)+np.square(vertical_dist))
    print(Real_dist)
    # Perform inference using YOLO model
    results = yolo(image_path)

    # Iterate over the detections
    for result in results:
        # Get class names
        classes_names = result.names

        # Iterate over each detected box
        for box in result.boxes:
            # Check if confidence is greater than 50%
            if box.conf[0] > 0.7:
                # Get coordinates of bounding box
                [x1, y1, x2, y2] = box.xyxy[0].tolist()
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                # Get the class ID
                cls = int(box.cls[0])

                # Get the class name
                class_name = classes_names[cls]

                # Process only if the detected object is a backboard
                if class_name.lower() == "backboard":
                    # Calculate width and height of the bounding box
                    bounding_box_width = abs(x1 - x2)
                    bounding_box_height = abs(y1 - y2)
                    bounding_box_diag = np.sqrt(np.square(bounding_box_width) + np.square(bounding_box_height))

                    # Calculate distance
                    distance = calculate_distance(KNOWN_DIAG, FOCAL_LENGTH, bounding_box_diag)

                    # Draw the rectangle around the backboard
                    colour = getColours(cls)
                    cv2.rectangle(orig, (x1, y1), (x2, y2), colour, 2)

                    # Put the class name and confidence on the image
                    cv2.putText(orig, f'{class_name} {box.conf[0]:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, colour, 2)

                    # Display the distance to the backboard
                    cv2.putText(orig, f'Distance: {distance:.2f} m', (x1, y2 + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, colour, 2)

    # Show the image with bounding boxes and information
    cv2.imshow('Detected Backboard', orig)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Provide the path to your image
image_path = "7.125m1.jpg"  # Replace with your image path
# Call the function to detect and highlight backboard
detect_and_highlight_backboard(image_path)
