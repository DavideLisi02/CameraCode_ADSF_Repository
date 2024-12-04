import os
import serial
import time

os.system('cls')  # Clear the terminal screen

os.system('cls')  # Clear the terminal screen

# Set up the serial connection to the ESP32
esp32 = serial.Serial(port='COM9', baudrate=512000, timeout=1)
time.sleep(2)  # Allow time for ESP32 reset

while True:
    try:
        # Get user input for motor positions
        motor1_pos = input('Enter position for motor1 (0-180), or type "exit" to quit: ')
        if motor1_pos.lower() == 'exit':
            print("Exiting...")
            break
        
        motor2_pos = input('Enter position for motor2 (0-180), or type "exit" to quit: ')
        if motor2_pos.lower() == 'exit':
            print("Exiting...")
            break

        # Validate inputs
        motor1_pos = int(motor1_pos)
        motor2_pos = int(motor2_pos)
        if not (0 <= motor1_pos <= 180) or not (0 <= motor2_pos <= 180):
            print("Invalid input. Both positions must be between 0 and 180.")
            continue

        # Send formatted message
        msgWR = f"motor1:{motor1_pos} motor2:{motor2_pos}\n"
        print(f"Sending: {msgWR}")
        esp32.write(bytes(msgWR, 'utf-8'))

        # Wait for ESP32 response
        time.sleep(0.5)
        if esp32.in_waiting > 0:
            msgRD = esp32.readline().decode('utf-8').strip()
            print(f"Received: {msgRD}")
        else:
            print("No response received from ESP32.")

    except ValueError:
        print("Invalid input. Please enter valid numbers between 0 and 180.")

esp32.close()
