import os
import serial
import time

os.system('cls')  # Clear the terminal screen

# Set up the serial connection to the ESP32
esp32 = serial.Serial(port='COM9', baudrate=512000, timeout=1)
time.sleep(2)  # Allow time for ESP32 reset

while True:

    # Send formatted message
    msgWR = f"motor1:{95} motor2:{95}\n"
    print(f"Sending: {msgWR}")
    esp32.write(bytes(msgWR, 'utf-8'))

    # Wait for ESP32 response
    time.sleep(0.5)
    if esp32.in_waiting > 0:
        msgRD = esp32.readline().decode('utf-8').strip()
        print(f"Received: {msgRD}")
    else:
        print("No response received from ESP32.")

    # Send formatted message
    msgWR = f"motor1:{85} motor2:{85}\n"
    print(f"Sending: {msgWR}")
    esp32.write(bytes(msgWR, 'utf-8'))

    # Wait for ESP32 response
    time.sleep(0.5)
    if esp32.in_waiting > 0:
        msgRD = esp32.readline().decode('utf-8').strip()
        print(f"Received: {msgRD}")
    else:
        print("No response received from ESP32.")


esp32.close()