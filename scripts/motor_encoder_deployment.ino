/*
 * AMR motor and encoder interface
 *
 * Serial protocol at 115200 baud:
 *   Jetson -> Arduino: CMD,<front_left>,<front_right>,<rear_left>,<rear_right>\n
 *   Arduino -> Jetson: ENC,<front_left>,<front_right>,<rear_left>,<rear_right>\n
 *
 * Wheel commands are signed PWM values in the range -255..255. A command
 * timeout stops the robot if the Jetson connection or ROS node fails.
 */

#include <Arduino.h>

// Motor pins (Arduino Mega)
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

// Encoder pins
#define ENC_A_A 18
#define ENC_A_B 31
#define ENC_B_A 19
#define ENC_B_B 38
#define ENC_C_A 3
#define ENC_C_B 49
#define ENC_D_A 2
#define ENC_D_B A1

const unsigned long COMMAND_TIMEOUT_MS = 500;
const unsigned long ENCODER_REPORT_PERIOD_MS = 50;
const size_t COMMAND_BUFFER_SIZE = 64;

volatile long encA = 0;
volatile long encB = 0;
volatile long encC = 0;
volatile long encD = 0;

char commandBuffer[COMMAND_BUFFER_SIZE];
size_t commandLength = 0;
bool commandOverflow = false;
unsigned long lastCommandTime = 0;
unsigned long lastEncoderReport = 0;

void setMotor(int pwm, uint8_t pwmPin, uint8_t dir1, uint8_t dir2,
              bool forwardHigh) {
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

void setWheelCommands(int fl, int fr, int rl, int rr) {
  // A and C are mounted with the opposite direction logic from B and D.
  setMotor(fl, PWMA, DIRA1, DIRA2, true);
  setMotor(fr, PWMB, DIRB1, DIRB2, false);
  setMotor(rl, PWMC, DIRC1, DIRC2, true);
  setMotor(rr, PWMD, DIRD1, DIRD2, false);
}

void stopAll() {
  setWheelCommands(0, 0, 0, 0);
}

void isrEncA() {
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

bool parseCommand(char *line) {
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

  setWheelCommands((int)values[0], (int)values[1], (int)values[2],
                   (int)values[3]);
  lastCommandTime = millis();
  return true;
}

void readCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      if (!commandOverflow) {
        commandBuffer[commandLength] = '\0';
        parseCommand(commandBuffer);
      }
      commandLength = 0;
      commandOverflow = false;
    } else if (commandLength < COMMAND_BUFFER_SIZE - 1) {
      commandBuffer[commandLength++] = c;
    } else {
      // Discard an overlong line and wait for its terminator.
      commandOverflow = true;
    }
  }
}

void reportEncoders() {
  noInterrupts();
  long fl = encA;
  long fr = encB;
  long rl = encC;
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

  lastCommandTime = millis();
  lastEncoderReport = millis();
}

void loop() {
  readCommands();

  unsigned long now = millis();
  if (now - lastCommandTime >= COMMAND_TIMEOUT_MS) {
    stopAll();
  }
  if (now - lastEncoderReport >= ENCODER_REPORT_PERIOD_MS) {
    lastEncoderReport = now;
    reportEncoders();
  }
}
