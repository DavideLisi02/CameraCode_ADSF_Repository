import cv2
import numpy as np

# Function to find the robot's position, orientation, and size in the image
def findRobot(img, video):
    # Create a binary mask to isolate white regions in the image within defined range
    mask = cv2.inRange(img, WHITE_LOWER, WHITE_UPPER)

    # Apply Gaussian blur to smooth out noise in the mask
    blurred_img = cv2.GaussianBlur(mask, (7, 7), 0)  # (7,7) chosen to reduce noise

    # Detect edges in the blurred image using the Canny edge detection method
    edges = cv2.Canny(blurred_img, 100, 255)  # Thresholds (100, 255) set experimentally

    # Find contours in the edge-detected image
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize an empty list to store results for each detected robot contour
    results = []

    # Iterate over each detected contour
    for contour in contours:
        # Calculate the contour area
        area = cv2.contourArea(contour)

        # Check if the contour area is within the specified range for identifying the robot
        if 30000 < area < 60000:
            # Fit a rotated rectangle around the contour, returns center, size, and angle
            rect = cv2.minAreaRect(contour)

            # Get the 4 corner points of the rectangle
            box = cv2.boxPoints(rect)
            box = np.int0(box)  # Convert points to integer for display

            # Draw the rectangle (representing the robot) on the video frame
            cv2.drawContours(video, [box], 0, (0, 0, 255), 3)

            # Extract center, angle, width, and height of the rectangle
            center = (int(rect[0][0]), int(rect[0][1]))
            angle = int(rect[2])
            width, height = (int(rect[1][0]), int(rect[1][1]))

            # Append the detected robot's attributes to the results list
            results.append((center, -angle, (width, height), box))

    # Return the binary mask and detected results
    return mask, results

# Function to detect the goal's position, orientation, and size in the image
def findGoal(img, video):
    # Create a binary mask to isolate yellow regions (goal) in the image
    mask = cv2.inRange(img, YELLOW_LOWER, YELLOW_UPPER)

    # Apply Gaussian blur to the mask to reduce noise
    blurred_img = cv2.GaussianBlur(mask, (7, 7), 0)

    # Detect edges in the blurred image using the Canny method
    edges = cv2.Canny(blurred_img, 100, 255)

    # Find contours in the edge-detected image
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize an empty list to store results for each detected goal contour
    results = []

    # Iterate over each detected contour
    for contour in contours:
        # Calculate the area of the contour
        area = cv2.contourArea(contour)

        # Check if the contour area is within the specified range for the goal
        if 23000 < area < 60000:
            # Fit a rotated rectangle around the contour
            rect = cv2.minAreaRect(contour)

            # Get the 4 corner points of the rectangle
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            # Draw the rectangle (representing the goal) on the video frame
            cv2.drawContours(video, [box], 0, (0, 255, 0), 3)

            # Extract center, angle, width, and height of the rectangle
            center = (int(rect[0][0]), int(rect[0][1]))
            angle = int(rect[2])
            width, height = (int(rect[1][0]), int(rect[1][1]))

            # Append the detected goal's attributes to the results list
            results.append((center, -angle, (width, height), box))

    # Return the binary mask and detected results
    return mask, results

# Function to find the angle and relative position between two squares (a larger and smaller one)
def findAngle(img, video):
    # Create a binary mask to isolate yellow regions in the image
    mask = cv2.inRange(img, YELLOW_LOWER, YELLOW_UPPER)

    # Apply Gaussian blur to reduce noise
    blurred_img = cv2.GaussianBlur(mask, (5, 5), 0)

    # Detect edges in the blurred image
    edges = cv2.Canny(blurred_img, 100, 255)

    # Find contours in the edge-detected image
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize variables to store the centers of two squares
    center_little_square, center_big_square = None, None

    # Iterate over each detected contour
    for contour in contours:
        # Calculate the area of the contour
        area = cv2.contourArea(contour)

        # Identify the larger square by area range
        if 14000 < area < 22000:
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            # Draw the rectangle on the video frame
            cv2.drawContours(video, [box], 0, (255, 0, 0), 3)

            # Store the center of the larger square
            center_big_square = (int(rect[0][0]), int(rect[0][1]))

        # Identify the smaller square by area range
        if 4000 < area < 12000:
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            # Draw the rectangle on the video frame
            cv2.drawContours(video, [box], 0, (255, 0, 0), 3)

            # Store the center of the smaller square
            center_little_square = (int(rect[0][0]), int(rect[0][1]))

    # Return the centers of the two squares
    return center_big_square, center_little_square

# Function to convert pixel location to millimeters using a ratio
def positionToMM(results, results_goal):
    # Calculate the conversion ratio from pixels to millimeters based on goal dimensions
    ratio = PIXEL2MM(results_goal)

    # Convert the x and y coordinates from pixels to millimeters
    x_mm = results[0][0][0] * ratio
    y_mm = results[0][0][1] * ratio

    # Return the converted x and y coordinates
    return x_mm, y_mm

# Function to calculate the pixel-to-millimeter ratio based on the goal width
def PIXEL2MM(results_goal):
    # If no goal detected, print message and return None
    if not results_goal:
        print("No object detected.")
        return None

    # Get the width of the goal in pixels
    width = results_goal[0][2][0]

    # Calculate the conversion ratio based on actual goal width in mm
    ratio = GOAL_WIDTH_MM / width

    # Return the calculated ratio
    return ratio