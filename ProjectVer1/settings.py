import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import loop
import threading
import queue

# Global variables for threshold values
AREA_MIN = 100
AREA_MAX = 3000
THRESHOLD_MIN = 50
THRESHOLD_MAX = 255
area_value_min = 100
area_value_max = 3000
threshold_value_min = 50
threshold_value_max = 255


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
            try:
                ip_parts = ip.split('.')
                if len(ip_parts) != 4 or not all(part.isdigit() and 0 <= int(part) <= 255 for part in ip_parts):
                    raise ValueError("Invalid IP address format")
                if not stream_port.isdigit():
                    raise ValueError("Invalid port number")
            except ValueError as e:
                messagebox.showerror("Error", str(e))
                return
            URL = f"{ip}:{stream_port}/stream"
        self.destroy()
        q = queue.Queue()
        thread = threading.Thread(target=loop.start_video_stream_fun, args=(URL, self.max_features.get(), self.good_match_percent.get(), THRESHOLD_MIN, THRESHOLD_MAX, AREA_MIN, AREA_MAX, q), daemon=False)
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
