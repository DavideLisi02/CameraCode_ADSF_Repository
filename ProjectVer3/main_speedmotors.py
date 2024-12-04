import threading
import cv2
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import threading
import queue
import serial
import time

# Global variables for threshold values
area_value_min = 100
area_value_max = 3000
threshold_value_min = 50
threshold_value_max = 255


####################################################################################################
####################################################################################################

#! Settings for control loop

# Com port for serial comunication with the esp32
com_port = 'COM9'

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
###CONTROL FUNCTIONS####################################################################################

def move_motors(i1,i2,esp32):
    '''
    This function send values to a microcontroller by serial port.
    
    Input:
    i1 : integer value, value to control motor1
    i2 : integer value, value to control motor2
    esp32 : object, variable related to the microcontroller in use
    
    Output:
    [i1,i2] : list
    '''

    msgWR = f"motor1:{i1} motor2:{i2}"
    print(f"THIS INPUT WAS GIVEN TO THE MOTORS:\nMotor1: {i1}\nMotor2: {i2}")
    print(f"Sending message to motors: {msgWR}")  # Debug: Print the message being sent to ESP32
    esp32.write(bytes(msgWR, 'utf-8'))  # Write the input message to the ESP32
    return [i1, i2]


def control_motors(dx,dy,esp32):
    
    i_1 = int(dx * A_1)
    i_2 = int(dy * A_2)

    if i_1>LOW_constr_1 and i_2>LOW_constr_2:
        i_1 = min((i_1,UP_constr_1))
        i_2 = min(i_2,UP_constr_2)
    elif i_1>LOW_constr_1 and i_2<LOW_constr_2:
        i_1 = min((i_1,UP_constr_1))
        i_2 = LOW_constr_2
    else:
        i_1 = LOW_constr_1
        i_2 = min(i_2,UP_constr_2)
        
    msgWR = f"motor1:{i_1} motor2:{i_2}"
    print(f"THIS INPUT WAS GIVEN TO THE MOTORS:\nMotor1: {i_1}\nMotor2: {i_2}")
    print(f"Sending message to motors: {msgWR}")  # Debug: Print the message being sent to ESP32
    esp32.write(bytes(msgWR, 'utf-8'))  # Write the input message to the ESP32
    return [i_1,i_2]




#### CAMERA FUNCTIONS ##########################################################################################
import cv2
import numpy as np

class VideoCapture:
    def __init__(self, name):
        self.cap = cv2.VideoCapture(name)

    def read(self):
        success, frame = self.cap.read()
        return success, frame

def find_reflection(image_0, image_1, threshold_value_min, threshold_value_max, min_area, max_area):
    gray_0 = cv2.cvtColor(image_0, cv2.COLOR_BGR2GRAY)
    gray_1 = cv2.cvtColor(image_1, cv2.COLOR_BGR2GRAY)
    diff = cv2.absdiff(gray_0, gray_1)
    _, thresh = cv2.threshold(diff, threshold_value_min, threshold_value_max, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid_contours = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if min_area <= area <= max_area:
            valid_contours.append(contour)
    reflection_x = None
    reflection_y = None
    found = False
    if len(valid_contours) > 0:
        found = True
        moments = cv2.moments(valid_contours[0])
        reflection_x = int(moments['m10'] / moments['m00'])
        reflection_y = int(moments['m01'] / moments['m00'])
        print(f"Centroide medio del riflesso: X = {reflection_x} | Y = {reflection_y}")
    else:
        print("Nessun contorno valido trovato.")
    thresh_with_contours = np.copy(thresh)
    cv2.drawContours(thresh_with_contours, valid_contours, -1, (155), 2)

    return ((reflection_x, reflection_y), found, diff, thresh_with_contours, thresh_with_contours)


def align_images(im1, im2):
    # Currently not used
    MAX_FEATURES = 500
    GOOD_MATCH_PERCENT = 0.15
    im1Gray = cv2.cvtColor(im1, cv2.COLOR_BGR2GRAY)
    im2Gray = cv2.cvtColor(im2, cv2.COLOR_BGR2GRAY)
    orb = cv2.ORB_create(MAX_FEATURES)
    keypoints1, descriptors1 = orb.detectAndCompute(im1Gray, None)
    keypoints2, descriptors2 = orb.detectAndCompute(im2Gray, None)
    matcher = cv2.DescriptorMatcher_create(cv2.DESCRIPTOR_MATCHER_BRUTEFORCE_HAMMING)
    matches = matcher.match(descriptors1, descriptors2)
    if isinstance(matches, tuple):
        matches = list(matches)
    matches.sort(key=lambda x: x.distance, reverse=False)
    numGoodMatches = int(len(matches) * GOOD_MATCH_PERCENT)
    matches = matches[:numGoodMatches]
    imMatches = cv2.drawMatches(im1, keypoints1, im2, keypoints2, matches, None)
    cv2.imwrite("matches.jpg", imMatches)
    points1 = np.zeros((len(matches), 2), dtype=np.float32)
    points2 = np.zeros((len(matches), 2), dtype=np.float32)
    for i, match in enumerate(matches):
        points1[i, :] = keypoints1[match.queryIdx].pt
        points2[i, :] = keypoints2[match.trainIdx].pt
    h, mask = cv2.findHomography(points1, points2, cv2.RANSAC)
    height, width, channels = im2.shape
    im1Reg = cv2.warpPerspective(im1, h, (width, height))
    return im1Reg
##########################################################################################

### LOOP ##########################################################################################

# Event to signal thread termination
stop_event = threading.Event()

def start_video_stream_fun(URL, max_features, good_match_percent, threshold_min, threshold_max, area_min, area_max, q):
    esp32 = serial.Serial(port='COM9', baudrate=115200, timeout=1)
    time.sleep(2)  # Give the ESP32 time to reset and initialize
    
    print("Starting video stream...")
    restart_loop = False
    while not stop_event.is_set():
        try:
            cap = cv2.VideoCapture(URL)
            print(f"Attempting to open video source: {URL}")
            if not cap.isOpened():
                print(f"Could not open video source: {URL}")
                q.put({"error": f"Could not open video source: {URL}"})
                break
            
            [i_1,i_2] = move_motors(no_ref_motor1,no_ref_motor2,esp32)
            
            success, initial_frame = cap.read()
            
            print(f"Attempting to read initial frame: success={success}")
            if not success:
                print("Could not read frame from video source.")
                q.put({"error": "Could not read frame from video source."})
                break
            
            tracker = cv2.TrackerKCF_create()

            [i_1,i_2] = move_motors(yes_ref_motor1,yes_ref_motor2,esp32)

            bbox_initialized = False
            while not stop_event.is_set() and not restart_loop:
                print("Inside main loop...")
                success, frame = cap.read()
                if not success:
                    break
                frame_copy = frame.copy()
                if not bbox_initialized:
                    bbox = cv2.selectROI("Video Stream", frame, False)
                    try:
                        tracker.init(frame, bbox)
                        bbox_initialized = True
                    except Exception as e:
                        print(f"Tracker initialization failed: {e}")
                        q.put({"error": f"Tracker initialization failed: {e}"})
                        break

                success, bbox = tracker.update(frame)
                if success:
                    (x_p, y_p, w, h) = [int(v) for v in bbox]
                    cv2.rectangle(frame, (x_p, y_p), (x_p + w, y_p + h), (0, 255, 0), 2, 1)
                    print(f"Object detected at coord = X:{(x_p + w) / 2} | Y:{(y_p + h) / 2}")
                else:
                    cv2.putText(frame, "Tracking failure", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
                reflection_xy = find_reflection(initial_frame, frame_copy, threshold_value_min, threshold_value_max, area_value_min, area_value_max)
                if reflection_xy[1]:
                    cv2.circle(frame, reflection_xy[0], 10, (0, 0, 255), 2)
                    x_r = reflection_xy[0][0]
                    y_r = reflection_xy[0][1]
                    dx = x_r - x_p
                    dy = y_r - y_p
                    [i_1,i_2] = control_motors(dx,dy,esp32)
                    # Draw an arrow from (x_r, y_r) to (x_r + i_1, y_r + i_2)
                    cv2.arrowedLine(frame, (x_r, y_r), (x_r + int(i_1), y_r + int(i_2)), (255, 0, 0), 2)

                resized_gray_diff = cv2.resize(reflection_xy[2], (400, 400))
                resized_threshold = cv2.resize(reflection_xy[3], (400, 400))
                resized_frame = cv2.resize(frame, (400, 400))

                cv2.arrowedLine(resized_frame, (0, 0), (40, 0), (255, 0, 0), 2)  # x direction
                cv2.arrowedLine(resized_frame, (0, 0), (0, 40), (255, 0, 0), 2)  # y direction
                cv2.putText(resized_frame, 'x', (45, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA) # x label
                cv2.putText(resized_frame, 'y', (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA) # y label


                q.put({"frame": resized_frame, "gray_diff": resized_gray_diff, "threshold": resized_threshold})
                cv2.imshow("Gray scale difference", resized_gray_diff)
                cv2.imshow("Threshold", resized_threshold)
                cv2.imshow("Video Stream", resized_frame)
                cv2.moveWindow("Gray scale difference", 0, 0)
                cv2.moveWindow("Threshold", 420, 0)
                cv2.moveWindow("Video Stream", 840, 0)
                if cv2.waitKey(3) & 0xFF == ord('r'):
                    restart_loop = True
                    break
                if cv2.waitKey(3) & 0xFF == ord('q'):
                    stop_event.set()
                    break
            restart_loop = False
            bbox_initialized = False

            cap.release()
            cv2.destroyAllWindows()
        except Exception as e:
            print(f"An error occurred: {e}")
            q.put({"error": f"An error occurred: {e}"})
            break

    print("Video stream stopped.")
    q.put({"done": True})
##########################################################################################

### WINDOW MANAGEMENT ##########################################################################################


try:
    CAMERA_IP = "192.168.1.100"  # Replace with your camera's IP address
    CONTROLLER_PORT = 5000       # Replace with your controller's port
except Exception as e:
    print(f"Error loading settings: {e}")

class SettingsDialog(tk.Tk):
    def __init__(self): # Removed stop_event
        super().__init__()
        self.title("Video Stream Settings")

        # Center the window
        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        window_width = 400
        window_height = 350
        x = (screen_width - window_width) // 2
        y = (screen_height - window_height) // 2
        self.geometry(f"{window_width}x{window_height}+{x}+{y}")

        self.configure(bg="#f0f0f0")
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)

        # Initialize StringVars
        self.ip_var = tk.StringVar(value="http://192.168.4.87")
        self.stream_var = tk.StringVar(value="81")
        self.max_features = tk.IntVar(value=250)
        self.good_match_percent = tk.DoubleVar(value=0.15)
        self.use_webcam = tk.BooleanVar(value=False)

        # Widgets with padding and font
        label_font = ("Helvetica", 12)
        entry_font = ("Helvetica", 12)
        button_font = ("Helvetica", 12, "bold")

        #Improved layout and styling
        tk.Label(self, text="Video Stream Settings", bg="#f0f0f0", font=("Helvetica", 16, "bold")).grid(row=0, column=0, columnspan=2, pady=10)

        tk.Label(self, text="Camera IP:", bg="#f0f0f0", font=label_font).grid(row=1, column=0, sticky="w", padx=10, pady=5)
        tk.Entry(self, textvariable=self.ip_var, width=30, font=entry_font).grid(row=1, column=1, padx=10, pady=5)
        tk.Label(self, text="Stream port:", bg="#f0f0f0", font=label_font).grid(row=2, column=0, sticky="w", padx=10, pady=5)
        tk.Entry(self, textvariable=self.stream_var, width=30, font=entry_font).grid(row=2, column=1, padx=10, pady=5)
        tk.Label(self, text="Max Features:", bg="#f0f0f0", font=label_font).grid(row=3, column=0, sticky="w", padx=10, pady=5)
        tk.Entry(self, textvariable=self.max_features, width=30, font=entry_font).grid(row=3, column=1, padx=10, pady=5)
        tk.Label(self, text="Good Match Percent:", bg="#f0f0f0", font=label_font).grid(row=4, column=0, sticky="w", padx=10, pady=5)
        tk.Entry(self, textvariable=self.good_match_percent, width=30, font=entry_font).grid(row=4, column=1, padx=10, pady=5)
        tk.Checkbutton(self, text="Use Webcam", variable=self.use_webcam, bg="#f0f0f0", font=label_font).grid(row=5, column=0, columnspan=2, pady=10)
        tk.Button(self, text="Start Video Stream", command=self.start_video_stream_thread, bg="#4CAF50", fg="white", font=button_font).grid(row=6, column=0, columnspan=2, pady=10)

    def start_video_stream_thread(self):
        if self.use_webcam.get():
            URL = 1
        else:
            ip = self.ip_var.get()
            stream_port = self.stream_var.get()
            URL = f"{ip}:{stream_port}/stream"
        self.destroy()
        q = queue.Queue()
        thread = threading.Thread(target=start_video_stream_fun, args=(URL, self.max_features.get(), self.good_match_percent.get(), threshold_value_min, threshold_value_min, area_value_min, area_value_max, q), daemon=False)
        thread.start()
        threshold_adjuster = ReflectionTrackerAdjuster()
        threshold_adjuster.update_idletasks()
        x = threshold_adjuster.winfo_x()
        y = threshold_adjuster.winfo_y() + threshold_adjuster.winfo_height()
        threshold_adjuster.geometry(f"+{x}+{y}")
        threshold_adjuster.mainloop()


# Explicit export of SettingsDialog and ReflectionTrackerAdjuster
from tkinter import Tk
from tkinter import ttk
from tkinter import BooleanVar
from tkinter import DoubleVar
from tkinter import IntVar
from tkinter import StringVar

class ReflectionTrackerAdjuster(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Adjust Reflection Detection Values")
        self.geometry("300x400")

        global threshold_value_min, threshold_value_max, area_value_min, area_value_max

        self.threshold_var_min = tk.IntVar(value=threshold_value_min)
        tk.Label(self, text="Min Threshold Value:").pack()
        tk.Scale(self, variable=self.threshold_var_min, from_=0, to=255, orient=tk.HORIZONTAL, command=self.update_min_threshold).pack()

        self.threshold_var_max = tk.IntVar(value=threshold_value_max)
        tk.Label(self, text="Max Threshold Value:").pack()
        tk.Scale(self, variable=self.threshold_var_max, from_=0, to=255, orient=tk.HORIZONTAL, command=self.update_max_threshold).pack()

        self.area_var_min = tk.IntVar(value=area_value_min)
        tk.Label(self, text="Min Area Value:").pack()
        tk.Scale(self, variable=self.area_var_min, from_=10, to=5000, orient=tk.HORIZONTAL, command=self.update_min_area).pack()

        self.area_var_max = tk.IntVar(value=area_value_max)
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

__all__ = ["SettingsDialog", "ReflectionTrackerAdjuster"]


# Event to signal thread termination (must be defined here to be accessible to both main and settings)
stop_event = threading.Event()

def main():
    settings_dialog = SettingsDialog() # Pass stop_event
    settings_dialog.protocol("WM_DELETE_WINDOW")
    settings_dialog.mainloop()

if __name__ == "__main__":
    main()
