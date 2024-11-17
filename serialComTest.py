import os
import serial
import time

os.system('cls')  # Clear the terminal screen

# Set up the serial connection to the ESP32
esp32 = serial.Serial(port='COM9', baudrate=115200, timeout=1)
time.sleep(2)  # Give the ESP32 time to reset and initialize

# Start the loop to continuously send and receive data
while True:
    # Get user input for the servo position
    msgWR = input('Write message to ESP32 (servo position 0-180), or type "exit" to quit: ')

    # Exit condition to stop the loop
    if msgWR.lower() == 'exit':
        print("Exiting...")
        break

    print(f"Sending message: {msgWR}")  # Debug: Print the message being sent to ESP32
    esp32.write(bytes(msgWR, 'utf-8'))  # Write the input message to the ESP32

    # Wait for the response from the ESP32
    time.sleep(0.5)  # Add a small delay to allow ESP32 to process the input
    msgRD = esp32.readline()  # Read the response from the ESP32
    msgRD = msgRD.decode('utf-8').strip()  # Convert the bytes to string and remove any extra newlines or spaces

    # Print the received message
    print(f'Read message from ESP32: {msgRD}\n')

# Close the serial connection when done
esp32.close()
