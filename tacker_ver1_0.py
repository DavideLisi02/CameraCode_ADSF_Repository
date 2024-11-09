import tkinter as tk
from tkinter import simpledialog, messagebox
import cv2
import os
import numpy as np
import requests
import queue
import threading

class VideoCapture:
    def __init__(self, name):
        self.cap = cv2.VideoCapture(name)
        self.q = queue.Queue()
        t = threading.Thread(target=self._reader)
        t.daemon = True
        t.start()

    def _reader(self):
        while True:
            success, frame = self.cap.read()
            if not success:
                break
            if not self.q.empty():
                try:
                    self.q.get_nowait()
                except queue.Empty:
                    pass
            self.q.put(frame)

    def read(self):
        return self.q.get()

def find_reflection(image_0, image_1, threshold_value_min, threshold_value_max):
    gray_0 = cv2.cvtColor(image_0, cv2.COLOR_BGR2GRAY)
    gray_1 = cv2.cvtColor(image_1, cv2.COLOR_BGR2GRAY)
    
    diff = cv2.absdiff(gray_0, gray_1)
    _, thresh = cv2.threshold(diff, threshold_value_min, threshold_value_max, cv2.THRESH_BINARY)

    white_pixels = np.column_stack(np.where(thresh == 255))
    reflection_x = None
    reflection_y = None

    if white_pixels.size > 0:
        found = True
        reflection_x = int(np.mean(white_pixels[:, 1]))
        reflection_y = int(np.mean(white_pixels[:, 0]))
        print(f"Average position of reflection: X = {reflection_x} | Y = {reflection_y}")
    else:
        found = False
        print("No white pixels found in the image.")

    return ((reflection_x, reflection_y), found, diff, thresh)

class SettingsDialog(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Video Stream Settings")
        self.geometry("400x300")

        # IP Entry
        self.ip_var = tk.StringVar(value="http://192.168.121.87")
        tk.Label(self, text="Camera IP:").pack()
        tk.Entry(self, textvariable=self.ip_var, width=30).pack()

        # Stream Choice
        self.stream_var = tk.StringVar(value="81")
        tk.Label(self, text="Stream port:").pack()
        tk.Entry(self, textvariable=self.stream_var, width=30).pack()

        # Threshold Setting Min
        self.threshold_var_min = tk.IntVar(value=50)
        tk.Label(self, text="Min Threshold Value:").pack()
        tk.Scale(self, variable=self.threshold_var_min, from_=0, to=255, orient=tk.HORIZONTAL).pack()

        # Threshold Setting Min
        self.threshold_var_max = tk.IntVar(value=255)
        tk.Label(self, text="Max Threshold Value:").pack()
        tk.Scale(self, variable=self.threshold_var_max, from_=0, to=255, orient=tk.HORIZONTAL).pack()

        # Start Button
        tk.Button(self, text="Start Video Stream", command=self.start_video_stream).pack()

    def start_video_stream(self):
        ip = self.ip_var.get()
        stream_port = self.stream_var.get()
        threshold_value_min = self.threshold_var_min.get()
        threshold_value_max = self.threshold_var_max.get()
        
        # Validate IP format
        if not ip.startswith("http"):
            messagebox.showerror("Error", "Invalid IP format.")
            return
        
        URL = f"{ip}:{stream_port}/stream"
        self.destroy()  # Close settings dialog before starting stream
        start_video_stream(URL, threshold_value_min, threshold_value_max)

def show_connection_error():
    """Displays a window with 'Connection Failed' message and options to retry or terminate."""
    root = tk.Tk()
    root.title("Connection Error")
    root.geometry("300x150")

    # Add message and buttons
    message_label = tk.Label(root, text="Failed to connect.\nEnsure you are connected to the camera server.", pady=10)
    message_label.pack()

    # Set up button actions
    def on_retry():
        root.destroy()
        return True

    def on_terminate():
        root.destroy()
        return False

    # Create Retry and Terminate buttons
    retry_button = tk.Button(root, text="Retry", command=lambda: root.quit())
    retry_button.pack(side="left", padx=20, pady=20)
    terminate_button = tk.Button(root, text="Terminate", command=lambda: root.quit())
    terminate_button.pack(side="right", padx=20, pady=20)

    root.mainloop()
    return retry_button

def start_video_stream(URL, threshold_value_min, threshold_value_max):
    while True:
        cap = cv2.VideoCapture(URL)
        success, initial_frame = cap.read()

        # Check if VideoCapture successfully opened the URL
        if not cap.isOpened():
            # Show error window with retry and terminate options
            if not show_connection_error():
                return  # Terminate if user chooses to stop
            continue  # Retry connection if user chooses to continue

        # If connection is successful, continue with video processing
        success, initial_frame = cap.read()
        if not success:
            print("Failed to read the initial frame from the video stream.")
            cap.release()
            continue  # Retry connection if there's an issue with the initial frame

        # Proceed with video processing if connection is successful
        tracker = cv2.TrackerKCF_create()
        output_file = 'output_video.avi'
        frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = 20.0
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        video_stored = cv2.VideoWriter(output_file, fourcc, fps, (frame_width, frame_height))
        
        first_iteration = True

        while True:
            if cap.isOpened():
                success, frame = cap.read()
                
                if success and first_iteration:
                    init_box = cv2.selectROI("Video Stream", frame, False)
                    tracker.init(frame, init_box)
                    first_iteration = False

                if not success:
                    break

                reflection_xy = find_reflection(initial_frame, frame, threshold_value_min, threshold_value_max)
                if reflection_xy[1]:
                    cv2.circle(frame, reflection_xy[0], 10, (0, 0, 255), 2)

                success, bbox = tracker.update(frame)
                if success:
                    (x, y, w, h) = [int(v) for v in bbox]
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2, 1)
                    print(f"Object detected at coord = X:{(x + w) / 2} | Y:{(y + h) / 2}")
                else:
                    cv2.putText(frame, "Tracking failure", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

                cv2.imshow("Gray scale difference", reflection_xy[2])
                cv2.imshow("Threshold", reflection_xy[3])
                cv2.imshow("Video Stream", frame)

                # Move each window to a specific position
                cv2.moveWindow("Gray scale difference", 0, 0)  # Position at (0,0)
                cv2.moveWindow("Threshold", 400, 0)            # Position to the right of the first window
                cv2.moveWindow("Video Stream", 800, 0)    

                video_stored.write(frame)

                key = cv2.waitKey(3)
                if key == ord('q'):
                    break

        cap.release()
        video_stored.release()
        cv2.destroyAllWindows()
        break

if __name__ == '__main__':
    app = SettingsDialog()
    app.mainloop()