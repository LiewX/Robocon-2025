import cv2
from ultralytics import YOLO
import numpy as np
import math
import time
import matplotlib.pyplot as plt
import os

# ==========================================
# workflow: 
'''
'help me tidy my code up, format nicely and add comments' god bless this prompt
help 
Given
- pi cam's camera matrix
- yolo_segmentation model to identify corners of the backboard
- known size and height of the backboard, field
    
Goal  
- to find absolute position of robot (camera) with respect to the field (for either backboard)
- so the robot can calculate its trajectory of launching the bkb
- in livestream mode, but should also have an option to read from images in a directory 

Workflow: 
- extract camera parameters, run inferencing..? on the footage/images captured
- extract bounding box of the backboard
- map the orientation and size of the backboard to its absolute position, assuming zero offsets from the backboard to the field  
- map to world coords [WIP: DEBUGGING FOR THE LIFE OF ME]

Side quest: 
- plt gui to display where the robot could be relative to the backboard 

  
'''

# =============================================
# BACKGROUND INFORMATION
# =============================================
"""
The playing area is the grey ground, measuring 15 meters in length
and 8 meters in width. It is surrounded by a fence with a height of
10 cm and a width of 5 cm. The playing area includes the backboard
and the baskets. However, the outside of the fence and the back
side of the backboard are not considered part of the playing area.
"""
# =============================================
# MACROS
# =============================================
KNOWN_BB_HEIGHT = 1.83  # Backboard height (vertical length) in meters
KNOWN_BB_WIDTH = 1.05   # Backboard width (horizontal) in meters
KNOWN_BB_DIAG = np.sqrt(np.square(KNOWN_BB_WIDTH) + np.square(KNOWN_BB_WIDTH))  # Diagonal for scale estimation

# Field dimensions
KNOWN_FIELD_WIDTH = 8
KNOWN_FIELD_LENGTH = 15
KNOWN_FIELD_DIAG = np.sqrt(np.square(KNOWN_FIELD_WIDTH) + np.square(KNOWN_FIELD_LENGTH))

# CHANGE THESE MACROS FOR TESTING 
KNOWN_BASKET_HEIGHT = 3.05     # Basketball ring height (to be verified)
KNOWN_CAMERA_HEIGHT = 0.7      # Camera height off the ground
BB_TO_FENCE_DIST = 0           # Assuming backboard is flush with fence

# Misc settings
PRINT_FLAG = True
YOLO_MODEL_PATH = "./weights/best.pt"  # Trained segmentation model
CAMERA_MATRIX_PATH = "./calibration/parameters/camera_matrix.txt"  # Pi camera matrix file
VIDEO_PATH = "./train_files/videos/testvid4.mp4"  # Video input path
MAX_LIST_SIZE = 25


# =============================================
# estimate_backboard_pose()
# Estimate pose (position and rotation) of backboard from image corners
# using the known 3D coordinates of backboard corners
# =============================================
def estimate_backboard_pose(camera_matrix, image_points):
    # Define object points in 3D space (top-left, top-right, bottom-left, bottom-right)
    object_points = np.array([
        [-KNOWN_BB_WIDTH / 2,  KNOWN_BB_HEIGHT / 2, 0],
        [ KNOWN_BB_WIDTH / 2,  KNOWN_BB_HEIGHT / 2, 0],
        [-KNOWN_BB_WIDTH / 2, -KNOWN_BB_HEIGHT / 2, 0],
        [ KNOWN_BB_WIDTH / 2, -KNOWN_BB_HEIGHT / 2, 0],
    ], dtype=np.float32)

    # Convert input image points to float32
    image_points = np.array(image_points, dtype=np.float32)

    # ASSUMING NO DISTORTION 
    dist_coeffs = np.zeros((4,1))
    # SolvePnP finds the rotation and translation vector from 3D-2D correspondences
    success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
    
    return (rvec, tvec) if success else (None, None)

# =============================================
# calculate_distance()
# Simple pinhole distance estimation using object diagonal and bounding box diagonal
# =============================================
def calculate_distance(known_diag, focal_length, bounding_box_diag):
    x = (known_diag * focal_length) / bounding_box_diag
    return 0 if math.isnan(x) else x

# =============================================
# getColours()
# Assign visually distinct colors to different classes
# =============================================
def getColours(cls_num):
    base_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
    color_index = cls_num % len(base_colors)
    increments = [(1, -2, 1), (-2, 1, -1), (1, -1, 2)]
    color = [
        base_colors[color_index][i] + increments[color_index][i] * (cls_num // len(base_colors)) % 256
        for i in range(3)
    ]
    return tuple(color)

# =============================================
# estimate_corners_from_mask()
# Get polygonal corner estimates from segmentation mask
# =============================================
def estimate_corners_from_mask(mask):
    # Find external contours from mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # Take the largest contour and approximate polygon
    largest = max(contours, key=cv2.contourArea)
    epsilon = 0.02 * cv2.arcLength(largest, True)
    approx = cv2.approxPolyDP(largest, epsilon, True)

    # If not quadrilateral, fallback to rotated bounding box
    if len(approx) != 4:
        rect = cv2.minAreaRect(largest)
        box = cv2.boxPoints(rect)
        approx = np.intp(box)

    # Return 4 ordered corners
    pts = approx.reshape((4, 2))
    return order_points(pts)

# =============================================
# order_points()
# Ensure consistent corner order: TL, TR, BL, BR
# =============================================
def order_points(pts):
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect

# =============================================
# process_frame()
# Main YOLO + segmentation inference, corner tracking, visualization
# =============================================
def process_frame(frame, yolo, known_diag, focal_length,
                  TL_List, TR_List, BL_List, BR_List, MAX_LIST_SIZE):
    
    xy_center = None
    results = yolo.track(frame, stream=True, verbose=False, persist=False)

    for result in results:
        if result.masks is None:
            continue

        class_names = result.names

        for box, mask in zip(result.boxes, result.masks):
            if box.conf[0] < 0.7: continue

            [x1, y1, x2, y2] = map(int, box.xyxy[0])
            cls = int(box.cls[0])
            class_name = class_names[cls]

            if class_name.lower() != "backboard": continue

            # Compute distance using diagonal
            bbox_width = abs(x1-x2)
            bbox_height = abs(y1-y2)
            diag_pixels = np.hypot(bbox_width, bbox_height)
            distance = calculate_distance(known_diag, focal_length, diag_pixels)

            # Extract and simplify polygon mask to corner approximation
            contour = mask.xy[0]
            approx = cv2.approxPolyDP(contour, 20, True)
            poly = cv2.approxPolyDP(approx, 20, True)

            if len(poly) < 4: continue

            # Get object center and track corners relative to it
            xy_center = box.xywh[:, :2].cpu().numpy()[0]
            corners = poly[:4]

            if len(corners) < 4: continue
                
            # Track each corner's offset from center across frames
            TL_List.append(corners[0][0] - xy_center)
            TR_List.append(corners[1][0] - xy_center)
            BL_List.append(corners[2][0] - xy_center)
            BR_List.append(corners[3][0] - xy_center)

            for L in [TL_List, TR_List, BL_List, BR_List]:
                if len(L) > MAX_LIST_SIZE:
                    L.pop(0)

            # Apply median filter to smooth corner jitter
            def filtered_corner(lst):
                stack = np.vstack(lst)
                return [np.median(stack[:, 0]), np.median(stack[:, 1])]

            # According to KJ, the mapping for TL, TR, BL, BR may not align but it is what it is
            filtered_TL = filtered_corner(TL_List)
            filtered_TR = filtered_corner(TR_List)
            filtered_BL = filtered_corner(BL_List)
            filtered_BR = filtered_corner(BR_List)

            # Colour the corners (red) 
            def draw_corner(pt, color):
                x, y = int(pt[0] + xy_center[0]), int(pt[1] + xy_center[1])
                cv2.circle(frame, (x, y), 5, color, -1)

            # Draw original segmentation mask (yellow)
            for point in contour:
                cv2.circle(frame, (int(point[0]), int(point[1])), 2, (0, 255, 255), -1)

            # Draw filtered corners 
            # the colours doesnt align with intended labels as described before
            draw_corner(filtered_TL, (0, 255, 0))           # green 
            draw_corner(filtered_TR, (0, 0, 255))           # red
            draw_corner(filtered_BL, (255, 0, 255))         # pink
            draw_corner(filtered_BR, (255, 255, 255))       # white

            # Draw bounding box and label
            cv2.rectangle(frame, (x1, y1), (x2, y2), getColours(cls), 2)
            cv2.putText(frame, f'{class_name} {box.conf[0]:.2f}', (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, getColours(cls), 2)
            cv2.putText(frame, f'Distance: {distance:.2f} m', (x1, y2 + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, getColours(cls), 2)

    return frame, xy_center


# =============================================
# MAIN FUNCTION
# =============================================
def main():
    # Load segmentation model
    yolo = YOLO(YOLO_MODEL_PATH, "segment")

    # Load video input
    cap = cv2.VideoCapture(VIDEO_PATH)

    # Load calibrated camera matrix
    camera_matrix = np.loadtxt(CAMERA_MATRIX_PATH, delimiter=',')  
    focal_length_x = camera_matrix[0][0]

    # For filtering
    TL_List, TR_List, BL_List, BR_List = [], [], [], []

    while True:
        ret, frame = cap.read()
        if not ret: break

        # Resize or crop frame if needed
        processed_frame, xy_center = process_frame(frame, yolo, KNOWN_BB_DIAG, focal_length_x, TL_List, TR_List, BL_List, BR_List, MAX_LIST_SIZE)
        if xy_center is None:
            print("[Skip] Backboard not found in this frame.")
            continue

        if len(TL_List) == MAX_LIST_SIZE:
            # Compute averaged and absolute 2D image points of corners
            corners = []
            for lst in [TL_List, TR_List, BL_List, BR_List]:
                offset = np.median(np.vstack(lst), axis=0)
                abs_point = offset + xy_center
                corners.append(abs_point)

            # Estimate pose THIS NEEDS FIXING STILL 
            rvec, tvec = estimate_backboard_pose(camera_matrix, corners)

            if rvec is not None and tvec is not None:
                # Draw coordinate axes on the frame

                # Extract and print position
                x=0, y=0, z=0
                print(f"[Backboard Position] x={x:.2f} m, y={y:.2f} m, z={z:.2f} m")


        # Display output
        if PRINT_FLAG: 
            cv2.imshow("Backboard Tracker", processed_frame)
            key = cv2.waitKey(1)
            if key == ord("q"): break
            if key == ord("p"): cv2.waitKey(-1)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    print(os.getcwd())
    main()