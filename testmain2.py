import cv2
import numpy as np
from ultralytics import YOLO

# Load the model
yolo = YOLO('best.pt')

# Load the video capture
videoCap = cv2.VideoCapture("testvid4.mp4")

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

# Detect contours of the backboard using edge detection
def detect_contours(roi, frame, x1, y1):
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    #blur = cv2.GaussianBlur(gray,[3,3],2)
    edges = cv2.Canny(gray, 200, 350)
    #blur1 = cv2.GaussianBlur(edges,[15,15],0.5)
    #edges1 = cv2.Canny(blur1, 150, 250)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # Draw all detected contours
    for cnt in contours:
        adjusted_contour = cnt + [x1, y1]  # Adjust contour coordinates to match the original frame
        cv2.drawContours(frame, [adjusted_contour], -1, (0, 255, 255), 2)

while True:
    ret, frame = videoCap.read()
    if not ret:
        continue
    results = yolo.track(frame, stream=True)

    for result in results:
        # Get the class names
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
                    # Extract the region of interest (ROI)
                    roi = frame[y1:y2, x1:x2]

                    # Detect and draw contours
                    detect_contours(roi, frame, x1, y1)

                    # Draw the rectangle
                    colour = getColours(cls)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)

                    # Put the class name and confidence on the image
                    cv2.putText(frame, f'{class_name} {box.conf[0]:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, colour, 2)

    # Show the frame
    cv2.imshow('frame', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and destroy all windows
videoCap.release()
cv2.destroyAllWindows()
