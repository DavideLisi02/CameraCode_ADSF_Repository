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

// Variables to store desired servo positions
int motor1_pos = 90;
int motor2_pos = 90;

// Timing variables
unsigned long lastUpdateTime = 0;
const int updateInterval = 30;  // Update interval in milliseconds (33ms = ~30 updates per second)

void setup() {
  Serial.begin(250000);  // Increase baud rate for faster communication

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
  // Check for serial input
  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n');  // Read incoming message until newline character
    int temp_motor1_pos = -1, temp_motor2_pos = -1;

    // Parse the message in the format "motor1:X motor2:Y"
    int motor1_index = message.indexOf("motor1:");
    int motor2_index = message.indexOf("motor2:");
    if (motor1_index != -1 && motor2_index != -1) {
      temp_motor1_pos = message.substring(motor1_index + 7, message.indexOf(' ', motor1_index)).toInt();
      temp_motor2_pos = message.substring(motor2_index + 7).toInt();

      // Update motor positions if valid
      if (temp_motor1_pos >= 0 && temp_motor1_pos <= 180) {
        motor1_pos = temp_motor1_pos;
      } else {
        Serial.println("Invalid position for Servo 1.");
      }
      if (temp_motor2_pos >= 0 && temp_motor2_pos <= 180) {
        motor2_pos = temp_motor2_pos;
      } else {
        Serial.println("Invalid position for Servo 2.");
      }
    }
  }

  // Periodically update servo positions
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= updateInterval) {
    myservo.write(motor1_pos);
    myservo2.write(motor2_pos);

    // Debug output
    Serial.print("Servo 1 Position: ");
    Serial.println(motor1_pos);
    Serial.print("Servo 2 Position: ");
    Serial.println(motor2_pos);

    lastUpdateTime = currentTime;
  }
}
