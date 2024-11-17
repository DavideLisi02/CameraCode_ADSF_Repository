#include <ESP32Servo.h>

Servo servoBot;  // create servo object to control a servo
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
int servoBotpin = 10;  // Define the pin the servo is attached to
#endif

void setup() {
  Serial.begin(115200);  // Start serial communication

  // Initialize timers for PWM
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoBot.setPeriodHertz(50);  // Set servo PWM frequency
  servoBot.attach(servoBotpin, 500, 1833);  // Attach servo on pin 47 with PWM range
  servoBot.attach(servoBotpin, 500, 2500);
  servoBot.write(0);
}

void loop() {
  if (Serial.available() > 0) {
    String message = Serial.readString();  // Read the incoming message
    int pos = message.toInt();  // Convert the message to an integer

    // Validate the position
    if (pos >= 0 && pos <= 180) {
      int mappedPos = map(pos, 0, 180, 0, 270);
      servoBot.write(pos);  // Set servo to the received position
      delay(100);
      Serial.println(pos);  // Send confirmation back to the serial monitor
    } else {
      Serial.println("Invalid position. Please enter a value between 0 and 180.");
    }

    delay(1000);  // Wait for the servo to reach the position
  }
}
