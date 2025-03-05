import cv2
from ultralytics import YOLO
import numpy as np
# Load the model
yolo = YOLO('best.pt')

# Load the video capture
videoCap = cv2.VideoCapture("testvid4.mp4")

# Known dimensions of the backboard
KNOWN_WIDTH = 1.83  # in meters
KNOWN_HEIGHT = 1.10  # in meters
FOCAL_LENGTH = 800  # Estimated focal length in pixels (adjust based on calibration)

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
def calculate_distance(known_width, focal_length, bounding_box_width):
    """
    Calculate the distance to the backboard.
    :param known_width: Real-world width of the backboard (meters)
    :param focal_length: Focal length of the camera (pixels)
    :param bounding_box_width: Width of the detected backboard in pixels
    :return: Distance to the backboard in meters
    """
    return (known_width * focal_length) / bounding_box_width

while True:
    ret, frame = videoCap.read()
    if not ret:
        continue
    results = yolo.track(frame, stream=True)

    for result in results:
        # Get the classes names
        classes_names = result.names

        # Iterate over each box
        for box in result.boxes:
            # Check if confidence is greater than 50%
            if box.conf[0] > 0.5:
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
                    bounding_box_width = x2 - x1

                    # Calculate distance
                    distance = calculate_distance(KNOWN_WIDTH, FOCAL_LENGTH, bounding_box_width)

                    # Extract the region of interest (ROI)
                    roi = frame[y1:y2, x1:x2]

                    # Perform edge detection on the ROI
                    # blur_roi=cv2.GaussianBlur(roi,(5,5),1)
                    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

                    corners = cv2.goodFeaturesToTrack(gray_roi,20,0.95,50)
                    edges = cv2.Canny(gray_roi, 100, 200)
                    if corners is None:
                        pass
                    else:
                        corners = np.int8(corners)
                        
                        for i in corners:
                            x,y = i.ravel()
                            cv2.circle(edges,(x,y),3,255,-1)
                    
                    

                    

                    # Draw the rectangle
                    colour = getColours(cls)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)

                    # Put the class name, confidence, and distance on the image
                    cv2.putText(frame, f'{class_name} {box.conf[0]:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, colour, 2)
                    cv2.putText(frame, f'Distance: {distance:.2f} m', (x1, y2 + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, colour, 2)

                    # Display the edge-detected ROI
                    cv2.imshow("Edge Detection (ROI)", edges)

    # Show the frame
    cv2.imshow('frame', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and destroy all windows
videoCap.release()
cv2.destroyAllWindows()
