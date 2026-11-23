#include <Servo.h>

Servo testServo;

const uint8_t SERVO_PIN = A3;
const unsigned long HOLD_MS = 2000;

void setup() {
  testServo.attach(SERVO_PIN);
}

void loop() {
  testServo.write(0);
  delay(HOLD_MS);
  testServo.write(180);
  delay(HOLD_MS);
}
