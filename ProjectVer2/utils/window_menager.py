import threading
import tkinter as tk
from main import start_video_stream_fun

class SettingsDialog(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Video Stream Settings")
        self.geometry("400x300")

        # IP Entry
        self.ip_var = tk.StringVar(value="http://192.168.4.87")
        tk.Label(self, text="Camera IP:").pack()
        tk.Entry(self, textvariable=self.ip_var, width=30).pack()

        # Use Webcam Checkbox
        self.use_webcam_var = tk.BooleanVar(value=False)
        tk.Checkbutton(self, text="Use Webcam", variable=self.use_webcam_var).pack()

        # Webcam Entry
        self.webcam_var = tk.StringVar(value="0")
        tk.Label(self, text="Webcam used:").pack()
        tk.Entry(self, textvariable=self.webcam_var, width=30).pack()



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
        if self.use_webcam_var.get():
            URL = self.webcam_var.get()
        else:
            ip = self.ip_var.get()
            stream_port = self.stream_var.get()
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