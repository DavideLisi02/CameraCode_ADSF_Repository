import tkinter as tk
import cv2
import numpy as np
import threading
import os
import time
import serial

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

class VideoCapture:
    def __init__(self, name):
        self.cap = cv2.VideoCapture(name)

    def read(self):
        success, frame = self.cap.read()
        return success, frame

def move_motors_no_reflection():
    
    msgWR = f"motor1:{no_ref_motor1} motor2:{no_ref_motor2}"
    print(f"THIS INPUT WAS GIVEN TO THE MOTORS:\nMotor1: {no_ref_motor1}\nMotor2: {no_ref_motor2}")
    print(f"Sending message to motors: {msgWR}")  # Debug: Print the message being sent to ESP32
    esp32.write(bytes(msgWR, 'utf-8'))  # Write the input message to the ESP32
    return [no_ref_motor1, no_ref_motor2]

def move_motors_yes_reflection():
    msgWR = f"motor1:{yes_ref_motor1} motor2:{yes_ref_motor2}"
    print(f"THIS INPUT WAS GIVEN TO THE MOTORS:\nMotor1: {yes_ref_motor1}\nMotor2: {yes_ref_motor2}")
    print(f"Sending message to motors: {msgWR}")  # Debug: Print the message being sent to ESP32
    esp32.write(bytes(msgWR, 'utf-8'))  # Write the input message to the ESP32
    return [yes_ref_motor1, yes_ref_motor2]

def control_motors(dx,dy,i_01,i_02):
    di_1 = dx * A_1
    di_2 = dy * A_2
    i_11 = i_01 + di_1
    i_12 = i_02 + di_2
    if i_11>LOW_constr_1 and i_12>LOW_constr_2:
        i_11 = min((i_11,UP_constr_1))
        i_12 = min(i_12,UP_constr_2)
    elif i_11>LOW_constr_1 and i_12<LOW_constr_2:
        i_11 = min((i_11,UP_constr_1))
        i_12 = LOW_constr_2
    else:
        i_11 = LOW_constr_1
        i_12 = min(i_12,UP_constr_2)
    msgWR = f"motor1:{i_11} motor2:{i_12}"
    print(f"THIS INPUT WAS GIVEN TO THE MOTORS:\nMotor1: {i_11}\nMotor2: {i_12}")
    print(f"Sending message to motors: {msgWR}")  # Debug: Print the message being sent to ESP32
    esp32.write(bytes(msgWR, 'utf-8'))  # Write the input message to the ESP32
    return [i_11,i_12]

def align_images(im1, im2):
    
    MAX_FEATURES = 500
    GOOD_MATCH_PERCENT = 0.15

    # Convert images to grayscale
    im1Gray = cv2.cvtColor(im1, cv2.COLOR_BGR2GRAY)
    im2Gray = cv2.cvtColor(im2, cv2.COLOR_BGR2GRAY)
    
    # Detect ORB features and compute descriptors.
    orb = cv2.ORB_create(MAX_FEATURES)
    keypoints1, descriptors1 = orb.detectAndCompute(im1Gray, None)
    keypoints2, descriptors2 = orb.detectAndCompute(im2Gray, None)
    
    # Match features.
    matcher = cv2.DescriptorMatcher_create(cv2.DESCRIPTOR_MATCHER_BRUTEFORCE_HAMMING)
    matches = matcher.match(descriptors1, descriptors2)

    # Ensure `matches` is a list
    if isinstance(matches, tuple):
        matches = list(matches)

    # Sort matches by score
    matches.sort(key=lambda x: x.distance, reverse=False)
    
    # Remove not so good matches
    numGoodMatches = int(len(matches) * GOOD_MATCH_PERCENT)
    matches = matches[:numGoodMatches]
    
    # Draw top matches
    imMatches = cv2.drawMatches(im1, keypoints1, im2, keypoints2, matches, None)
    cv2.imwrite("matches.jpg", imMatches)
    
    # Extract location of good matches
    points1 = np.zeros((len(matches), 2), dtype=np.float32)
    points2 = np.zeros((len(matches), 2), dtype=np.float32)
    
    for i, match in enumerate(matches):
        points1[i, :] = keypoints1[match.queryIdx].pt
        points2[i, :] = keypoints2[match.trainIdx].pt
    
    # Find homography
    h, mask = cv2.findHomography(points1, points2, cv2.RANSAC)
    
    # Use homography
    height, width, channels = im2.shape
    im1Reg = cv2.warpPerspective(im1, h, (width, height))
    
    return im1Reg

def find_reflection(image_0, image_1, threshold_value_min, threshold_value_max, min_area, max_area):
    # Allinea le due immagini
    #image_1_aligned = align_images(image_0, image_1)
    
    # Converte le immagini in scala di grigi
    gray_0 = cv2.cvtColor(image_0, cv2.COLOR_BGR2GRAY)
    gray_1 = cv2.cvtColor(image_1, cv2.COLOR_BGR2GRAY)

    # Calcola la differenza assoluta tra le immagini
    diff = cv2.absdiff(gray_0, gray_1)
    
    # Applica la threshold per ottenere un'immagine binaria
    _, thresh = cv2.threshold(diff, threshold_value_min, threshold_value_max, cv2.THRESH_BINARY)

    # Trova i contorni nell'immagine binaria
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Lista per contenere i contorni che soddisfano i criteri di area
    valid_contours = []
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if min_area <= area <= max_area:
            valid_contours.append(contour)

    # Se ci sono contorni validi, calcola la posizione media
    reflection_x = None
    reflection_y = None
    found = False
    
    if len(valid_contours) > 0:
        found = True
        # Calcola la posizione media del centroide dei contorni validi
        moments = cv2.moments(valid_contours[0])
        reflection_x = int(moments['m10'] / moments['m00'])
        reflection_y = int(moments['m01'] / moments['m00'])
        
        print(f"Centroide medio del riflesso: X = {reflection_x} | Y = {reflection_y}")
    else:
        print("Nessun contorno valido trovato.")

        # Draw contours on the thresholded image
    thresh_with_contours = np.copy(thresh)  # Copy the thresh image to draw on
    cv2.drawContours(thresh_with_contours, valid_contours, -1, (155), 2)  # Draw contours in white (255) with thickness 2

    return ((reflection_x, reflection_y), found, diff, thresh_with_contours, thresh_with_contours)

class SettingsDialog(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Video Stream Settings")
        self.geometry("400x300")

        # IP Entry
        self.ip_var = tk.StringVar(value="http://192.168.4.87")
        tk.Label(self, text="Camera IP:").pack()
        tk.Entry(self, textvariable=self.ip_var, width=30).pack()

        # Stream Choice
        self.stream_var = tk.StringVar(value="81")
        tk.Label(self, text="Stream port:").pack()
        tk.Entry(self, textvariable=self.stream_var, width=30).pack()

        # MAX FEATURES for align function global variable
        self.max_features = tk.IntVar(value=250)
        tk.Label(self, text="Max Features:").pack()
        tk.Entry(self, textvariable=self.max_features, width=30).pack()

        # GOOD MATCH PERCENT for align function global variable
        self.good_match_percent = tk.DoubleVar(value=0.15)
        tk.Label(self, text="Good Match Percent").pack()
        tk.Entry(self, textvariable=self.good_match_percent, width=30).pack()

        # Start Button
        tk.Button(self, text="Start Video Stream", command=self.start_video_stream_thread).pack()

    def start_video_stream_thread(self):
        ip = self.ip_var.get()
        stream_port = self.stream_var.get()
        
        # Validate IP format
        if not ip.startswith("http"):
            tk.messagebox.showerror("Error", "Invalid IP format.")
            return
        
        URL = f"{ip}:{stream_port}/stream"
        self.destroy()  # Close settings dialog before starting stream

        # Start video stream in a separate thread to keep GUI responsive
        threading.Thread(target=start_video_stream_fun, args=(URL,), daemon=True).start()

class ReflectionTrackerAdjuster(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Adjust Reflection Detection Values")
        self.geometry("300x400")

        global threshold_value_min, threshold_value_max

        # Threshold Setting Min
        self.threshold_var_min = tk.IntVar(value=50)
        tk.Label(self, text="Min Threshold Value:").pack()
        tk.Scale(self, variable=self.threshold_var_min, from_=0, to=255, orient=tk.HORIZONTAL, command=self.update_min_threshold).pack()

        # Threshold Setting Max
        self.threshold_var_max = tk.IntVar(value=255)
        tk.Label(self, text="Max Threshold Value:").pack()
        tk.Scale(self, variable=self.threshold_var_max, from_=0, to=255, orient=tk.HORIZONTAL, command=self.update_max_threshold).pack()

        # Area Setting Min
        self.area_var_min = tk.IntVar(value=50)
        tk.Label(self, text="Min Area Value:").pack()
        tk.Scale(self, variable=self.area_var_min, from_=10, to=5000, orient=tk.HORIZONTAL, command=self.update_min_area).pack()

        # Area Setting Max
        self.area_var_max = tk.IntVar(value=3000)
        tk.Label(self, text="Max Area Value:").pack()
        tk.Scale(self, variable=self.area_var_max, from_=50, to=100000, orient=tk.HORIZONTAL, command=self.update_max_area).pack()

    def update_min_threshold(self, val):
        global threshold_value_min
        threshold_value_min = int(val)

    def update_max_threshold(self, val):
        global threshold_value_max
        threshold_value_max = int(val)

    def update_min_area(self, val):
        global area_value_min
        area_value_min = int(val)

    def update_max_area(self, val):
        global area_value_max
        area_value_max = int(val)

def start_video_stream_fun(URL):
    
    continue_streaming = True #Controls the following cycle while. if someone press r it enables to restart the whole code, take a new image and run again the code

    while continue_streaming:    
        [i_1,i_2] = move_motors_no_reflection()
        cap = cv2.VideoCapture(URL)
        success, initial_frame = cap.read()
        [i_1,i_2] = move_motors_yes_reflection()
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


if __name__ == '__main__':
    app = SettingsDialog()
    threshold_adjuster = ReflectionTrackerAdjuster()  # Start the threshold adjustment window
    threading.Thread(target=threshold_adjuster.mainloop, daemon=True).start()  # Run the sliders in a separate thread
    app.mainloop()
