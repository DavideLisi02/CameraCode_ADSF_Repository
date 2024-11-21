import threading
import os
from window_menager import *

os.system('cls')  # Clear the terminal screen

# Set up the serial connection to the ESP32
#esp32 = serial.Serial(port=com_port, baudrate=115200, timeout=1)
#time.sleep(2)  # Give the ESP32 time to reset and initialize

if __name__ == '__main__':
    app = SettingsDialog()
    threshold_adjuster = ReflectionTrackerAdjuster()  # Start the threshold adjustment window
    threading.Thread(target=threshold_adjuster.mainloop, daemon=True).start()  # Run the sliders in a separate thread
    app.mainloop()