import tkinter as tk
import cv2
import numpy as np
import threading
import os
import time
import serial
from camera_functions import *
from control_functions import *

####################################################################################################
####################################################################################################

# Constraints for motor movement
UP_constr_1 = 120
LOW_constr_1 = 0
UP_constr_2 = 180
LOW_constr_2 = 0

# No reflection position of the motors
no_ref_motor1 = 0
no_ref_motor2 = 180

# Reflection ensured position of the motors
yes_ref_motor1 = 20
yes_ref_motor2 = 150

# Constants for control
# P component A = [A_1 0, 0 A_2]
A_1 = 0.005
A_2 = 0.005

####################################################################################################
####################################################################################################

def start_video_stream_fun(URL):
    '''correct mistakes here!!!!'''
    continue_streaming = True #Controls the following cycle while. if someone press r it enables to restart the whole code, take a new image and run again the code

    while continue_streaming:    
        [i_1,i_2] = move_motors()
        cap = cv2.VideoCapture(URL)
        success, initial_frame = cap.read()
        [i_1,i_2] = move_motors()
        if not cap.isOpened():
            print("Failed to connect to the video stream.")
            return

        # Initialize the tracker
        tracker = cv2.TrackerKCF_create()  # Use KCF tracker (adjust based on OpenCV version)
        bbox_initialized = False

        while True:
            success, frame = cap.read()
            if not success:
                break

            # Create a copy of the frame before any drawing operations
            frame_copy = frame.copy()

            # Initialize the bounding box for tracking on the first frame
            if not bbox_initialized:
                bbox = cv2.selectROI("Video Stream", frame, False)
                tracker.init(frame, bbox)
                bbox_initialized = True

            # Update tracker
            success, bbox = tracker.update(frame)
            if success:
                (x_p, y_p, w, h) = [int(v) for v in bbox]
                cv2.rectangle(frame, (x_p, y_p), (x_p + w, y_p + h), (0, 255, 0), 2, 1)
                print(f"Object detected at coord = X:{(x_p + w) / 2} | Y:{(y_p + h) / 2}")
            else:
                cv2.putText(frame, "Tracking failure", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

            # Reflection detection using thresholded differences on unaltered frame_copy
            reflection_xy = find_reflection(initial_frame, frame_copy, threshold_value_min, threshold_value_max, area_value_min, area_value_max)
            if reflection_xy[1]:
                cv2.circle(frame, reflection_xy[0], 10, (0, 0, 255), 2)
                x_r = reflection_xy[0][0]
                y_r = reflection_xy[0][1]
                dx = x_r - x_p
                dy = y_r - y_p
                [i_1,i_2] = control_motors(dx,dy,i_1,i_2)
                # Draw an arrow from (x_r, y_r) to (x_r + i_1, y_r + i_2)
                cv2.arrowedLine(frame, (x_r, y_r), (x_r + int(i_1), y_r + int(i_2)), (255, 0, 0), 2)

            # Display images with updated threshold in real-time
            resized_gray_diff = cv2.resize(reflection_xy[2], (400, 400))  # Resize as needed
            resized_threshold = cv2.resize(reflection_xy[3], (400, 400))
            resized_frame = cv2.resize(frame, (400, 400))

            # Show the resized images in separate windows
            cv2.imshow("Gray scale difference", resized_gray_diff)
            cv2.imshow("Threshold", resized_threshold)
            cv2.imshow("Video Stream", resized_frame)

            # Move each window to a specific position on the screen
            cv2.moveWindow("Gray scale difference", 0, 0)
            cv2.moveWindow("Threshold", 420, 0)
            cv2.moveWindow("Video Stream", 840, 0)

            key = cv2.waitKey(3)
            if key == ord('q'):
                continue_streaming = False
                break
            if key == ord('r'):
                continue_streaming = True
                break
            
            time.sleep(2)

        cap.release()
        cv2.destroyAllWindows()

# Global variables for threshold values
threshold_value_min = 50
threshold_value_max = 255
# Global variables for area reflection values
area_value_min = 100
area_value_max = 3000
# Global variables for alignement of images
MAX_FEATURES = 500
GOOD_MATCH_PERCENT = 0.15

os.system('cls')  # Clear the terminal screen

# Set up the serial connection to the ESP32
esp32 = serial.Serial(port='COM9', baudrate=115200, timeout=1)
time.sleep(2)  # Give the ESP32 time to reset and initialize

if __name__ == '__main__':
    app = SettingsDialog()
    threshold_adjuster = ReflectionTrackerAdjuster()  # Start the threshold adjustment window
    threading.Thread(target=threshold_adjuster.mainloop, daemon=True).start()  # Run the sliders in a separate thread
    app.mainloop()
