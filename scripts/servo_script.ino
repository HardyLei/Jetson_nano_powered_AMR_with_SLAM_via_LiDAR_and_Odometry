#include <Servo.h>

Servo myServo;

const int SERVO_PIN = A5;  // signal wire (orange) on A5

void setup() {
  Serial.begin(9600);
  myServo.attach(SERVO_PIN);
  myServo.write(0);        // start at 0 degrees
  Serial.println("Servo ready at 0 degrees.");
  Serial.println("Type 1 to go to 90 degrees, 0 to go back to 0 degrees.");
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '1') {
      myServo.write(90);
      Serial.println("Moved to 90 degrees");
    } else if (c == '0') {
      myServo.write(0);
      Serial.println("Moved to 0 degrees");
    }
  }
}
