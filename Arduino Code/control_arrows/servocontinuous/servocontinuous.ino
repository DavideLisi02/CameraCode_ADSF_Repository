#include <ESP32Servo.h>

Servo servo1;
Servo servo2;

// Initial servo angles
int angle1 = 90; // Neutral position
int angle2 = 90;

void setup() {
  servo1.attach(10, 500, 2500); // Attach servo1 to GPIO 10 with min/max pulse widths
  servo2.attach(11, 500, 2500); // Attach servo2 to GPIO 11 with min/max pulse widths
  servo1.write(angle1); // Set initial position
  servo2.write(angle2);

  Serial.begin(512000); // Start serial communication at 512000 baud rate
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read(); // Read a single character command

    switch (command) {
      case 'w': // Up arrow
        angle2 = constrain(angle2 + 5, 0, 180);
        servo2.write(angle2);
        Serial.println("Servo2 up");
        break;
      case 's': // Down arrow
        angle2 = constrain(angle2 - 5, 0, 180);
        servo2.write(angle2);
        Serial.println("Servo2 down");
        break;
      case 'a': // Left arrow
        angle1 = constrain(angle1 - 5, 0, 180);
        servo1.write(angle1);
        Serial.println("Servo1 left");
        break;
      case 'd': // Right arrow
        angle1 = constrain(angle1 + 5, 0, 180);
        servo1.write(angle1);
        Serial.println("Servo1 right");
        break;
    }
  }
}
