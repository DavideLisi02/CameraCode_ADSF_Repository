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

void setup() {
  Serial.begin(115200);

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
  myservo.write(90);
  myservo2.write(90);
}

void loop() {
  if (Serial.available() > 0) {
    String message = Serial.readString();  // Read the incoming message
    int motor1_pos = -1, motor2_pos = -1;

    // Parse the message in the format "motor1:X motor2:Y"
    int motor1_index = message.indexOf("motor1:");
    int motor2_index = message.indexOf("motor2:");
    if (motor1_index != -1 && motor2_index != -1) {
      motor1_pos = message.substring(motor1_index + 7, message.indexOf(' ', motor1_index)).toInt();
      motor2_pos = message.substring(motor2_index + 7).toInt();
    }

    // Control motor 1
    if (motor1_pos >= 0 && motor1_pos <= 180) {
      myservo.write(motor1_pos);
      delay(100);
      myservo.write(90);
      Serial.print("Servo 1 Position: ");
      Serial.println(motor1_pos);
    } else {
      Serial.println("Invalid position for Servo 1.");
    }

    // Control motor 2
    if (motor2_pos >= 0 && motor2_pos <= 180) {
      myservo2.write(motor2_pos);
      delay(100);
      myservo2.write(90);
      Serial.print("Servo 2 Position: ");
      Serial.println(motor2_pos);
    } else {
      Serial.println("Invalid position for Servo 2.");
    }
  }

  delay(20);  // Short delay for stability
}
