import threading
import cv2
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import threading
import queue
import serial
import time
from simple_pid import PID
import os


# Global variables for threshold values
area_value_min = 1037
area_value_max = 100000
threshold_value_min = 19
threshold_value_max = 255

previous_thread_finished = False

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
no_ref_motor1 = 90
no_ref_motor2 = 100

# Reflection ensured position of the motors
yes_ref_motor1 = 90
yes_ref_motor2 = 80

# Constants for control
Kp_1 = 1
Ki_1 = 0
Kd_1 = 0
Kp_2 = 1
Ki_2 = 0
Kd_2 = 0

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
    print("[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]")
    msgWR = f"motor1:{i1} motor2:{i2}"
    print(f"THIS INPUT WAS GIVEN TO THE MOTORS:\nMotor1: {i1}\nMotor2: {i2}")
    print(f"Sending message to motors: {msgWR}")  # Debug: Print the message being sent to ESP32
    esp32.write(bytes(msgWR, 'utf-8'))  # Write the input message to the ESP32
    time.sleep(0.5)
    if esp32.in_waiting > 0:
        msgRD = esp32.readline().decode('utf-8').strip()
        print(f"Received: {msgRD}")
    else:
        print("No response received from ESP32.")
    return [i1, i2]


def control_motors(x_p,y_p,x_r,y_r,esp32):
    
    #i_1 = pid_control(Kp_1, Ki_1, Kd_1, x_p, x_r, (LOW_constr_1,UP_constr_1))
    #i_2 = pid_control(Kp_2, Ki_2, Kd_2, y_p, y_r, (LOW_constr_2,UP_constr_2))
    print(f"INSIDE CONTOL_MOTORS")

    i_1 = int(0.05*(x_p-x_r)+90)
    i_2 = int(0.05*(y_p-y_r)+90)

    if i_1>LOW_constr_1 and i_2>LOW_constr_2:
        i_1 = min((i_1,UP_constr_1))
        i_2 = min(i_2,UP_constr_2)
    elif i_1>LOW_constr_1 and i_2<LOW_constr_2:
        i_1 = min((i_1,UP_constr_1))
        i_2 = LOW_constr_2
    else:
        i_1 = LOW_constr_1
        i_2 = min(i_2,UP_constr_2)

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
    print(found)
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

def start_video_stream_fun(URL, q, desired_fps=10, sleep_time=0.5): # Added sleep_time parameter
    esp32 = serial.Serial(port='COM9', baudrate=512000, timeout=1)
    time.sleep(2)
    print("Starting video stream...")
    restart_loop = False
    while not stop_event.is_set():
        try:
            [i_1,i_2] = move_motors(no_ref_motor1,no_ref_motor2,esp32)

            time.sleep(2)
            cap = cv2.VideoCapture(URL)
            print(f"Attempting to open video source: {URL}")
            if not cap.isOpened():
                print(f"Could not open video source: {URL}")
                q.put({"error": f"Could not open video source: {URL}"})
                break
        
            success, initial_frame = cap.read()
            
            print(f"Attempting to read initial frame: success={success}")
            if not success:
                print("Could not read frame from video source.")
                q.put({"error": "Could not read frame from video source."})
                break
            
            tracker = cv2.TrackerKCF_create()
            [i_1,i_2] = move_motors(yes_ref_motor1,yes_ref_motor2,esp32)

            time.sleep(1)
            bbox_initialized = False
            i=0
            #main loop
            while not stop_event.is_set() and not restart_loop:
                print("Inside main loop...")
                success, frame = cap.read()
                if not success:
                    break
                if not success:
                    print("Could not read frame from video source.")
                    q.put({"error": "Could not read frame from video source."})
                    break
                frame_copy = frame.copy()
                if not bbox_initialized:
                    bbox = cv2.selectROI("Video Stream", frame, False)
                    tracker.init(frame, bbox)
                    bbox_initialized = True

                success, bbox = tracker.update(frame)
                if success:
                    (x_b, y_b, w, h) = [int(v) for v in bbox]
                    cv2.rectangle(frame, (x_b, y_b), (x_b + w, y_b + h), (0, 255, 0), 2, 1)
                    print(f"Object detected at coord = X:{(x_b + w) / 2} | Y:{(y_b + h) / 2}")
                    x_p = x_b+w/2
                    y_p = y_b+h/2
                else:
                    cv2.putText(frame, "Tracking failure", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
                reflection_xy = find_reflection(initial_frame, frame_copy, threshold_value_min, threshold_value_max, area_value_min, area_value_max)
                if reflection_xy[1]:
                    cv2.circle(frame, reflection_xy[0], 10, (0, 0, 255), 2)
                    x_r = reflection_xy[0][0]
                    y_r = reflection_xy[0][1]
                    [i_1,i_2] = control_motors(x_p, y_p, x_r, y_r, esp32)
                    move_motors(i_1,i_2,esp32)                 
                    # Draw an arrow from (x_r, y_r) to (x_r + i_1, y_r + i_2)
                    cv2.arrowedLine(frame, (x_r, y_r), (x_r + i_1 -90, y_r + i_2-90), (255, 0, 0), 2)

                gray_diff = cv2.resize(reflection_xy[2], (400, 400))
                threshold = cv2.resize(reflection_xy[3], (400, 400))

                cv2.arrowedLine(frame, (0, 0), (40, 0), (255, 0, 0), 2)  # x direction
                cv2.arrowedLine(frame, (0, 0), (0, 40), (255, 0, 0), 2)  # y direction
                cv2.putText(frame, 'x', (45, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA) # x label
                cv2.putText(frame, 'y', (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA) # y label

                cv2.imshow("Gray scale difference", gray_diff)
                cv2.imshow("Threshold", threshold)
                cv2.imshow("Video Stream", frame)
                cv2.moveWindow("Gray scale difference", 0, 0)
                cv2.moveWindow("Threshold", 420, 0)
                cv2.moveWindow("Video Stream", 840, 0)
                i+=1
                if cv2.waitKey(3) & 0xFF == ord('r'):
                    restart_loop = True
                    break
                if cv2.waitKey(3) & 0xFF == ord('q'):
                    stop_event.set()
                    break
                #time.sleep(sleep_time) # Added sleep
            restart_loop = False
            bbox_initialized = False

            cap.release()
            cv2.destroyAllWindows()
        except Exception as e:
            print(f"An error occurred: {e}")
            q.put({"error": f"An error occurred: {e}"})
            break
    esp32.close()
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
        self.ip_var = tk.StringVar(value="http://192.168.59.87")
        self.stream_var = tk.StringVar(value="81")
        self.use_webcam = tk.BooleanVar(value=False)
        self.fps_var = tk.IntVar(value=10) # Added fps variable
        self.sleep_var = tk.DoubleVar(value=0.5) # Added sleep variable

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
        tk.Checkbutton(self, text="Use Webcam", variable=self.use_webcam, bg="#f0f0f0", font=label_font).grid(row=5, column=0, columnspan=2, pady=10)
        tk.Label(self, text="FPS:", bg="#f0f0f0", font=label_font).grid(row=6, column=0, sticky="w", padx=10, pady=5)
        tk.Entry(self, textvariable=self.fps_var, width=30, font=entry_font).grid(row=6, column=1, padx=10, pady=5)
        tk.Label(self, text="Sleep Time (s):", bg="#f0f0f0", font=label_font).grid(row=7, column=0, sticky="w", padx=10, pady=5)
        tk.Entry(self, textvariable=self.sleep_var, width=30, font=entry_font).grid(row=7, column=1, padx=10, pady=5)
        tk.Button(self, text="Start Video Stream", command=self.start_video_stream_thread, bg="#4CAF50", fg="white", font=button_font).grid(row=8, column=0, columnspan=2, pady=10)

    def start_video_stream_thread(self):
        if self.use_webcam.get():
            URL = 1
        else:
            ip = self.ip_var.get()
            stream_port = self.stream_var.get()
            URL = f"{ip}:{stream_port}/stream"
        self.destroy()
        q = queue.Queue()
        desired_fps = self.fps_var.get()
        sleep_time = self.sleep_var.get()
        start_video_stream_fun(URL, q, desired_fps, sleep_time)


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
