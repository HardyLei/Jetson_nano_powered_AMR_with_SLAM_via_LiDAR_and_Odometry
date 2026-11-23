/*
 * AMR motor and encoder interface
 *
 * Serial protocol at 115200 baud:
 *   Jetson -> Arduino: CMD,<front_left>,<front_right>,<rear_left>,<rear_right>\n
 *   Jetson -> Arduino: DOOR,OPEN\n
 *   Arduino -> Jetson: ENC,<front_left>,<front_right>,<rear_left>,<rear_right>\n
 *
 * Wheel commands are signed PWM values in the range -255..255. A command
 * timeout stops the robot if the Jetson connection or ROS node fails.
 */

#include <Arduino.h>
#include <Servo.h>

// Motor driver pins (Arduino Mega). Each wheel has one PWM pin and two
// direction pins.
#define PWMA 12
#define DIRA1 34
#define DIRA2 35
#define PWMB 8
#define DIRB1 37
#define DIRB2 36
#define PWMC 9
#define DIRC1 43
#define DIRC2 42
#define PWMD 5
#define DIRD1 A4
#define DIRD2 A5

// Encoder A pins for each wheel. Channel A triggers the interrupt; channel B
// determines the direction of the count.
#define ENC_A_A 18
#define ENC_A_B 31
#define ENC_B_A 19
#define ENC_B_B 38
#define ENC_C_A 3
#define ENC_C_B 49
#define ENC_D_A 2
#define ENC_D_B A1

const uint8_t DOOR_SERVO_PIN = A3;
const int DOOR_OPEN_ANGLE = 126;
const unsigned long DOOR_OPEN_MS = 5000;
Servo doorServo;
bool doorOpen = false;
unsigned long doorOpenedAt = 0;

// Stop the motors when commands have not arrived within this interval.
const unsigned long COMMAND_TIMEOUT_MS = 500;
// Send encoder feedback to the Jetson at 20 Hz.
const unsigned long ENCODER_REPORT_PERIOD_MS = 50;
// Maximum length of one complete serial command, including its terminator.
const size_t COMMAND_BUFFER_SIZE = 64;

// Closed-loop speed control, matching motor_encoder_test.ino.
// The controller compares encoder counts measured during each 50 ms window
// with the calibrated target and changes PWM proportionally to the error.
const unsigned long SPEED_CONTROL_INTERVAL_MS = 50;
const int TARGET_COUNTS_PER_INTERVAL = 115;
const float SPEED_CONTROL_KP = 0.9f;
// The right-side target is raised slightly to compensate for chassis drift.
const float STEER_TRIM = 0.04f;

// Encoder counters are volatile because they are updated inside interrupt
// service routines.
volatile long encA = 0;
volatile long encB = 0;
volatile long encC = 0;
volatile long encD = 0;

// requestedPwm* stores the latest signed command from the Jetson.
// pwm* stores the magnitude currently used after feedback correction.
int requestedPwmA = 0, requestedPwmB = 0;
int requestedPwmC = 0, requestedPwmD = 0;
int pwmA = 0, pwmB = 0, pwmC = 0, pwmD = 0;
bool motorAActive = false, motorBActive = false;
bool motorCActive = false, motorDActive = false;
// The previous encoder samples define the next control window.
unsigned long lastSpeedControl = 0;
long lastControlA = 0, lastControlB = 0;
long lastControlC = 0, lastControlD = 0;
bool speedControlInitialized = false;

char commandBuffer[COMMAND_BUFFER_SIZE];
size_t commandLength = 0;
bool commandOverflow = false;
unsigned long lastCommandTime = 0;
unsigned long lastEncoderReport = 0;

void writeMotor(int pwm, uint8_t pwmPin, uint8_t dir1, uint8_t dir2,
                bool forwardHigh) {
  // Convert a signed PWM command into direction outputs and a non-negative
  // duty cycle. The two motor pairs use opposite physical direction wiring.
  // true: positive PWM selects dir1 HIGH/dir2 LOW;
  // false: positive PWM selects the inverted direction.
  pwm = constrain(pwm, -255, 255);
  if (pwm == 0) {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, LOW);
    analogWrite(pwmPin, 0);
    return;
  }

  bool forward = pwm > 0;
  if (!forwardHigh) {
    forward = !forward;
  }
  digitalWrite(dir1, forward ? HIGH : LOW);
  digitalWrite(dir2, forward ? LOW : HIGH);
  analogWrite(pwmPin, abs(pwm));
}

void setMotor(int commandPwm, int &requestedPwm, int &outputPwm,
              bool &active, uint8_t pwmPin, uint8_t dir1, uint8_t dir2,
              bool forwardHigh) {
  commandPwm = constrain(commandPwm, -255, 255);
  bool wasActive = active;
  bool commandChanged = requestedPwm != commandPwm;
  bool directionChanged = (requestedPwm < 0) != (commandPwm < 0);
  requestedPwm = commandPwm;
  active = commandPwm != 0;

  // A new command magnitude or direction establishes the new control output.
  // Repeated copies of the same command leave the feedback correction intact.
  if (!active || !wasActive || commandChanged || directionChanged) {
    outputPwm = abs(commandPwm);
  }
  writeMotor(active ? (commandPwm < 0 ? -outputPwm : outputPwm) : 0,
             pwmPin, dir1, dir2, forwardHigh);
}

void setWheelCommands(int fl, int fr, int rl, int rr) {
  // Map protocol order (front-left, front-right, rear-left, rear-right) to
  // the four motor drivers. A and C are mounted opposite to B and D.
  setMotor(fl, requestedPwmA, pwmA, motorAActive, PWMA, DIRA1, DIRA2, true);
  setMotor(fr, requestedPwmB, pwmB, motorBActive, PWMB, DIRB1, DIRB2, false);
  setMotor(rl, requestedPwmC, pwmC, motorCActive, PWMC, DIRC1, DIRC2, true);
  setMotor(rr, requestedPwmD, pwmD, motorDActive, PWMD, DIRD1, DIRD2, false);
}

void stopAll() {
  setWheelCommands(0, 0, 0, 0);
}

void isrEncA() {
  // Quadrature decoding: equal channel levels represent one direction;
  // unequal levels represent the other.
  encA += (digitalRead(ENC_A_A) == digitalRead(ENC_A_B)) ? -1 : 1;
}

void isrEncB() {
  encB += (digitalRead(ENC_B_A) == digitalRead(ENC_B_B)) ? -1 : 1;
}

void isrEncC() {
  encC += (digitalRead(ENC_C_A) == digitalRead(ENC_C_B)) ? -1 : 1;
}

void isrEncD() {
  encD += (digitalRead(ENC_D_A) == digitalRead(ENC_D_B)) ? -1 : 1;
}

void readNormalizedCounts(long &a, long &b, long &c, long &d) {
  // Copy all counters atomically so an interrupt cannot update a value
  // halfway through a read. Sign changes make forward motion positive.
  noInterrupts();
  a = -encA;
  b = encB;
  c = -encC;
  d = encD;
  interrupts();
}

int adjustPwm(int pwm, long measuredCounts, int targetCounts) {
  // Positive error means the wheel is too slow, so increase its duty cycle.
  long error = targetCounts - measuredCounts;
  pwm += (int)(SPEED_CONTROL_KP * error);
  return constrain(pwm, 0, 255);
}

void updateSpeedControl() {
  unsigned long now = millis();
  if (now - lastSpeedControl < SPEED_CONTROL_INTERVAL_MS) {
    return;
  }
  lastSpeedControl = now;

  long a, b, c, d;
  readNormalizedCounts(a, b, c, d);
  if (!speedControlInitialized) {
    lastControlA = a;
    lastControlB = b;
    lastControlC = c;
    lastControlD = d;
    speedControlInitialized = true;
    return;
  }

  // Use the absolute count change because the same speed target applies in
  // either commanded direction.
  long deltaA = labs(a - lastControlA);
  long deltaB = labs(b - lastControlB);
  long deltaC = labs(c - lastControlC);
  long deltaD = labs(d - lastControlD);
  // Apply steering trim only to the right-side wheels.
  int targetLeft = TARGET_COUNTS_PER_INTERVAL;
  int targetRight = (int)(TARGET_COUNTS_PER_INTERVAL * (1.0f + STEER_TRIM));

  if (motorAActive) {
    pwmA = adjustPwm(pwmA, deltaA, targetLeft);
    writeMotor(requestedPwmA < 0 ? -pwmA : pwmA, PWMA, DIRA1, DIRA2, true);
  }
  if (motorBActive) {
    pwmB = adjustPwm(pwmB, deltaB, targetRight);
    writeMotor(requestedPwmB < 0 ? -pwmB : pwmB, PWMB, DIRB1, DIRB2, false);
  }
  if (motorCActive) {
    pwmC = adjustPwm(pwmC, deltaC, targetLeft);
    writeMotor(requestedPwmC < 0 ? -pwmC : pwmC, PWMC, DIRC1, DIRC2, true);
  }
  if (motorDActive) {
    pwmD = adjustPwm(pwmD, deltaD, targetRight);
    writeMotor(requestedPwmD < 0 ? -pwmD : pwmD, PWMD, DIRD1, DIRD2, false);
  }

  lastControlA = a;
  lastControlB = b;
  lastControlC = c;
  lastControlD = d;
}

bool parseCommand(char *line) {
  // Split a complete line and require exactly: CMD,<4 signed PWM values>.
  char *fields[5];
  char *field = strtok(line, ",");
  size_t fieldCount = 0;
  while (field != NULL && fieldCount < 5) {
    fields[fieldCount++] = field;
    field = strtok(NULL, ",");
  }
  if (field != NULL || fieldCount != 5 || strcmp(fields[0], "CMD") != 0) {
    return false;
  }

  char *end = NULL;
  long values[4];
  for (size_t i = 0; i < 4; ++i) {
    values[i] = strtol(fields[i + 1], &end, 10);
    if (*fields[i + 1] == '\0' || *end != '\0' || values[i] < -255 ||
        values[i] > 255) {
      return false;
    }
  }

  // Only refresh the timeout after a syntactically valid command.
  setWheelCommands((int)values[0], (int)values[1], (int)values[2],
                   (int)values[3]);
  lastCommandTime = millis();
  return true;
}

void readCommands() {
  // Accumulate input until newline so partial USB serial packets are safe.
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      if (!commandOverflow) {
        commandBuffer[commandLength] = '\0';
        if (strcmp(commandBuffer, "DOOR,OPEN") == 0) {
          if (!doorOpen) {
            doorServo.write(DOOR_OPEN_ANGLE);
            doorOpenedAt = millis();
            doorOpen = true;
          }
        } else {
          parseCommand(commandBuffer);
        }
      }
      commandLength = 0;
      commandOverflow = false;
    } else if (commandLength < COMMAND_BUFFER_SIZE - 1) {
      commandBuffer[commandLength++] = c;
    } else {
      // Discard an overlong line and wait for its terminator before accepting
      // another command.
      commandOverflow = true;
    }
  }
}

void reportEncoders() {
  // Snapshot and transmit cumulative counts; the Jetson uses these for
  // odometry and monitoring.
  noInterrupts();
  // Normalize encoder signs so forward wheel motion is positive.
  long fl = -encA;
  long fr = encB;
  long rl = -encC;
  long rr = encD;
  interrupts();

  Serial.print(F("ENC,"));
  Serial.print(fl);
  Serial.print(',');
  Serial.print(fr);
  Serial.print(',');
  Serial.print(rl);
  Serial.print(',');
  Serial.println(rr);
}

void setup() {
  Serial.begin(115200);

  // Configure the motor driver outputs before stopping all wheels.
  pinMode(PWMA, OUTPUT);
  pinMode(DIRA1, OUTPUT);
  pinMode(DIRA2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRB1, OUTPUT);
  pinMode(DIRB2, OUTPUT);
  pinMode(PWMC, OUTPUT);
  pinMode(DIRC1, OUTPUT);
  pinMode(DIRC2, OUTPUT);
  pinMode(PWMD, OUTPUT);
  pinMode(DIRD1, OUTPUT);
  pinMode(DIRD2, OUTPUT);
  stopAll();

  // Pull-ups keep the encoder inputs in a defined state when idle.
  pinMode(ENC_A_A, INPUT_PULLUP);
  pinMode(ENC_A_B, INPUT_PULLUP);
  pinMode(ENC_B_A, INPUT_PULLUP);
  pinMode(ENC_B_B, INPUT_PULLUP);
  pinMode(ENC_C_A, INPUT_PULLUP);
  pinMode(ENC_C_B, INPUT_PULLUP);
  pinMode(ENC_D_A, INPUT_PULLUP);
  pinMode(ENC_D_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A_A), isrEncA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_A), isrEncB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_C_A), isrEncC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_D_A), isrEncD, CHANGE);

  doorServo.write(0);
  doorServo.attach(DOOR_SERVO_PIN);

  // Do not trigger the safety timeout immediately after startup.
  lastCommandTime = millis();
  lastEncoderReport = millis();
}

void loop() {
  // Service all complete serial commands without blocking the control loop.
  readCommands();

  unsigned long now = millis();
  if (doorOpen && now - doorOpenedAt >= DOOR_OPEN_MS) {
    doorServo.write(0);
    doorOpen = false;
  }
  if (now - lastCommandTime >= COMMAND_TIMEOUT_MS) {
    // Fail safe if the Jetson or its ROS node stops communicating.
    stopAll();
  }
  // Update motor duty cycles independently of the serial command rate.
  updateSpeedControl();
  if (now - lastEncoderReport >= ENCODER_REPORT_PERIOD_MS) {
    lastEncoderReport = now;
    reportEncoders();
  }
}
