import serial
import keyboard
import time

# Configure serial communication
ser = serial.Serial(port='COM9', baudrate=512000, timeout=1)  # Ensure 'COM9' matches your ESP32 port
time.sleep(2)  # Allow ESP32 to initialize

print("Use arrow keys to control the servos. Press 'q' to quit.")

try:
    while True:
        # Check for key presses
        if keyboard.is_pressed('up'):
            ser.write(b'w')  # Send 'w' to ESP32
            time.sleep(0.1)  # Small delay to avoid flooding
        elif keyboard.is_pressed('down'):
            ser.write(b's')  # Send 's' to ESP32
            time.sleep(0.1)
        elif keyboard.is_pressed('left'):
            ser.write(b'a')  # Send 'a' to ESP32
            time.sleep(0.1)
        elif keyboard.is_pressed('right'):
            ser.write(b'd')  # Send 'd' to ESP32
            time.sleep(0.1)
        elif keyboard.is_pressed('q'):
            print("Exiting...")
            break
except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    ser.close()