#include <ESP32Servo.h>

Servo servoBot;   // Servo object for the first motor
Servo servoTop;   // Servo object for the second motor

int servoBotpin = 10; // Pin for the first servo
int servoToppin = 11; // Pin for the second servo

void setup() {
  Serial.begin(115200);  // Start serial communication

  // Initialize timers for PWM
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoBot.setPeriodHertz(50);  // Set PWM frequency for the first servo
  servoTop.setPeriodHertz(50);  // Set PWM frequency for the second servo

  servoBot.attach(servoBotpin, 500, 2500);  // Attach the first servo
  servoTop.attach(servoToppin, 500, 2500);  // Attach the second servo

  servoBot.write(0);  // Set initial position for the first servo
  servoTop.write(0);  // Set initial position for the second servo
}

void loop() {
  if (Serial.available() > 0) {
    String message = Serial.readString();  // Read the incoming message
    message.trim();  // Remove any extra whitespace

    // Parse the input for two commands
    int motor1Index = message.indexOf("motor1:");
    int motor2Index = message.indexOf("motor2:");
    int motor1Pos = -1;
    int motor2Pos = -1;

    if (motor1Index != -1) {
      int endIndex = message.indexOf(' ', motor1Index); // Find the end of motor1 value
      motor1Pos = message.substring(motor1Index + 7, endIndex).toInt();
    }

    if (motor2Index != -1) {
      motor2Pos = message.substring(motor2Index + 7).toInt();
    }

    // Validate and set positions for each motor
    if (motor1Pos >= 0 && motor1Pos <= 180) {
      servoBot.write(motor1Pos);
      Serial.print("Motor1 set to position: ");
      Serial.println(motor1Pos);
    } else if (motor1Pos != -1) {
      Serial.println("Invalid position for motor1.");
    }

    if (motor2Pos >= 0 && motor2Pos <= 180) {
      servoTop.write(motor2Pos);
      Serial.print("Motor2 set to position: ");
      Serial.println(motor2Pos);
    } else if (motor2Pos != -1) {
      Serial.println("Invalid position for motor2.");
    }

    delay(1000);  // Allow time for the servos to move
  }
}

