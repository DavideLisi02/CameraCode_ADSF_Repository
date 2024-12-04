#include <ESP32Servo.h>

Servo myservo;   // Create servo object for the first servo
Servo myservo2;  // Create servo object for the second servo

// Define pins
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
int servoPin = 10;
int servoPin2 = 11;
#else
int servoPin = 4;   // Fallback pin for generic ESP32
int servoPin2 = 5;  // Fallback pin for the second servo
#endif

// Variables to store servo positions
int motor1_pos = 90;  // Default to neutral position
int motor2_pos = 90;  // Default to neutral position

void setup() {
  Serial.begin(512000);

  // Allocate timers for the ESP32
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Initialize servos
  myservo.setPeriodHertz(50);   
  myservo.attach(servoPin, 500, 2500);
  myservo2.setPeriodHertz(50);
  myservo2.attach(servoPin2, 500, 2500);

  // Set initial positions
  myservo.write(motor1_pos);
  myservo2.write(motor2_pos);
}

void loop() {
  static String serialInput = "";  // Store incomplete commands

  // Process serial data
  while (Serial.available() > 0) {
    char incomingChar = Serial.read();  // Read the next character

    // Check for the end of the command
    if (incomingChar == '\n') {
      // Parse the completed command
      parseCommand(serialInput);
      serialInput = "";  // Reset the input buffer
    } else {
      serialInput += incomingChar;  // Add character to the input buffer
    }
  }
}

// Function to parse and execute the command
void parseCommand(const String &command) {
  int motor1_index = command.indexOf("motor1:");
  int motor2_index = command.indexOf("motor2:");

  if (motor1_index != -1) {
    int space_after_motor1 = command.indexOf(' ', motor1_index);
    String motor1_value = (space_after_motor1 != -1) 
                          ? command.substring(motor1_index + 7, space_after_motor1) 
                          : command.substring(motor1_index + 7);
    motor1_pos = motor1_value.toInt();
    if (motor1_pos >= 0 && motor1_pos <= 180) {
      myservo.write(motor1_pos);
      Serial.print("Updated Servo 1 Position: ");
      Serial.println(motor1_pos);
    } else {
      Serial.println("Invalid position for Servo 1.");
    }
  }

  if (motor2_index != -1) {
    String motor2_value = command.substring(motor2_index + 7);
    motor2_pos = motor2_value.toInt();
    if (motor2_pos >= 0 && motor2_pos <= 180) {
      myservo2.write(motor2_pos);
      Serial.print("Updated Servo 2 Position: ");
      Serial.println(motor2_pos);
    } else {
      Serial.println("Invalid position for Servo 2.");
    }
  }
}
