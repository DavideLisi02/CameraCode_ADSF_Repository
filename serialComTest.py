import os
import serial
import time

os.system('cls')  # Clear the terminal screen

# Set up the serial connection to the ESP32
esp32 = serial.Serial(port='COM9', baudrate=115200, timeout=1)
time.sleep(2)  # Give the ESP32 time to reset and initialize

# Start the loop to continuously send and receive data
while True:
    # Get user input for the servo positions
    try:
        motor1_pos = input('Enter position for motor1 (0-180), or type "exit" to quit: ')
        
        # Exit condition to stop the loop
        if motor1_pos.lower() == 'exit':
            print("Exiting...")
            break
        
        motor2_pos = input('Enter position for motor2 (0-180), or type "exit" to quit: ')
        
        # Exit condition for the second motor
        if motor2_pos.lower() == 'exit':
            print("Exiting...")
            break
        
        # Ensure inputs are integers and within range
        motor1_pos = int(motor1_pos)
        motor2_pos = int(motor2_pos)
        
        if not (0 <= motor1_pos <= 180) or not (0 <= motor2_pos <= 180):
            print("Invalid input. Both positions must be between 0 and 180.")
            continue

        # Format the message to match the ESP32 code
        msgWR = f"motor1:{motor1_pos} motor2:{motor2_pos}"

        print(f"Sending message: {msgWR}")  # Debug: Print the message being sent to ESP32
        esp32.write(bytes(msgWR, 'utf-8'))  # Write the input message to the ESP32

        # Wait for the response from the ESP32
        time.sleep(0.5)  # Add a small delay to allow ESP32 to process the input
        msgRD = esp32.readline()  # Read the response from the ESP32
        msgRD = msgRD.decode('utf-8').strip()  # Convert the bytes to string and remove any extra newlines or spaces

        # Print the received message
        print(f'Read message from ESP32: {msgRD}\n')

    except ValueError:
        print("Invalid input. Please enter valid numbers between 0 and 180.")

# Close the serial connection when done
esp32.close()
