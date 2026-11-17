/*
 * 4-Motor + Encoder Test
 * ----------------------
 * USB (115200 baud). Send single characters:
 * 
 * --- CHASSIS MOVEMENTS ---
 *     q = front left      w = advance         e = front right
 *     a = left            s = STOP            d = right
 *     z = left back       x = back            c = right back
 *     j = rotate left     k = rotate right
 * 
 * --- INDIVIDUAL MOTORS ---
 *     1 = Motor A (FL) Fwd     5 = Motor A (FL) Back
 *     2 = Motor B (FR) Fwd     6 = Motor B (FR) Back
 *     3 = Motor C (RL) Fwd     7 = Motor C (RL) Back
 *     4 = Motor D (RR) Fwd     8 = Motor D (RR) Back
 * 
 * Encoder counts for all 4 wheels are printed every 2500 ms.
 */

#include <Arduino.h>

// ---------------- Motor pins ----------------------------
#define PWMA   12
#define DIRA1  34
#define DIRA2  35
#define PWMB    8
#define DIRB1  37
#define DIRB2  36
#define PWMC    9
#define DIRC1  43
#define DIRC2  42
#define PWMD    5
#define DIRD1  A4
#define DIRD2  A5

// ---------------- Encoder pins --------------------------
#define ENC_A_A  18
#define ENC_A_B  31
#define ENC_B_A  19
#define ENC_B_B  38
#define ENC_C_A  3
#define ENC_C_B  49
#define ENC_D_A  2
#define ENC_D_B  A1

// ---------------- Speed ---------------------------------
// Open-loop calibration from encoder measurements at the same commanded speed.
// Re-test these values under load; closed-loop control is required for full
// compensation when the load or battery voltage changes.
const uint8_t DRIVE_PWM_A = 151;
const uint8_t DRIVE_PWM_B = 139;
const uint8_t DRIVE_PWM_C = 173;
const uint8_t DRIVE_PWM_D = 142;

// ---------------- Closed-loop speed control -------------
// The calibrated PWM values produce about 115 counts in 50 ms.
const unsigned long SPEED_CONTROL_INTERVAL_MS = 50;
const int TARGET_COUNTS_PER_INTERVAL = 115;
const float SPEED_CONTROL_KP = 0.9f;

// Steering trim for straight-line driving.
// The speed controller equalizes every wheel to the same encoder count, but
// unequal tire diameters, grip, or friction can still make the chassis veer.
// This raises (or lowers) the target speed of the RIGHT-side wheels (B and D)
// relative to the LEFT-side wheels (A and C):
//   STEER_TRIM > 0 -> right wheels faster -> car steers LEFT  (fixes drift to the RIGHT)
//   STEER_TRIM < 0 -> car steers RIGHT
// Tune in small steps (a few %) until the car tracks straight.
const float STEER_TRIM = 0.04f;   // +4% right-side target boost

bool motorAActive = false, motorBActive = false;
bool motorCActive = false, motorDActive = false;
int pwmA = DRIVE_PWM_A, pwmB = DRIVE_PWM_B;
int pwmC = DRIVE_PWM_C, pwmD = DRIVE_PWM_D;

unsigned long lastSpeedControl = 0;
long lastControlA = 0, lastControlB = 0;
long lastControlC = 0, lastControlD = 0;
bool speedControlInitialized = false;

void restartSpeedControl() {
  speedControlInitialized = false;
}

// ---------------- Encoder counters ----------------------
volatile long encA = 0, encB = 0, encC = 0, encD = 0;

// ---------------- Motor helper macros -------------------
// Note: DIRA and DIRC logic is inverted compared to B and D to account for physical motor mounting

void motorAForward() {
  digitalWrite(DIRA1, HIGH); digitalWrite(DIRA2, LOW);
  pwmA = DRIVE_PWM_A; analogWrite(PWMA, pwmA); motorAActive = true;
  restartSpeedControl();
}
void motorABackoff() {
  digitalWrite(DIRA1, LOW); digitalWrite(DIRA2, HIGH);
  pwmA = DRIVE_PWM_A; analogWrite(PWMA, pwmA); motorAActive = true;
  restartSpeedControl();
}
void motorAStop() {
  digitalWrite(DIRA1, LOW); digitalWrite(DIRA2, LOW);
  analogWrite(PWMA, 0); motorAActive = false; restartSpeedControl();
}

void motorBForward() {
  digitalWrite(DIRB1, LOW); digitalWrite(DIRB2, HIGH);
  pwmB = DRIVE_PWM_B; analogWrite(PWMB, pwmB); motorBActive = true;
  restartSpeedControl();
}
void motorBBackoff() {
  digitalWrite(DIRB1, HIGH); digitalWrite(DIRB2, LOW);
  pwmB = DRIVE_PWM_B; analogWrite(PWMB, pwmB); motorBActive = true;
  restartSpeedControl();
}
void motorBStop() {
  digitalWrite(DIRB1, LOW); digitalWrite(DIRB2, LOW);
  analogWrite(PWMB, 0); motorBActive = false; restartSpeedControl();
}

void motorCForward() {
  digitalWrite(DIRC1, HIGH); digitalWrite(DIRC2, LOW);
  pwmC = DRIVE_PWM_C; analogWrite(PWMC, pwmC); motorCActive = true;
  restartSpeedControl();
}
void motorCBackoff() {
  digitalWrite(DIRC1, LOW); digitalWrite(DIRC2, HIGH);
  pwmC = DRIVE_PWM_C; analogWrite(PWMC, pwmC); motorCActive = true;
  restartSpeedControl();
}
void motorCStop() {
  digitalWrite(DIRC1, LOW); digitalWrite(DIRC2, LOW);
  analogWrite(PWMC, 0); motorCActive = false; restartSpeedControl();
}

void motorDForward() {
  digitalWrite(DIRD1, LOW); digitalWrite(DIRD2, HIGH);
  pwmD = DRIVE_PWM_D; analogWrite(PWMD, pwmD); motorDActive = true;
  restartSpeedControl();
}
void motorDBackoff() {
  digitalWrite(DIRD1, HIGH); digitalWrite(DIRD2, LOW);
  pwmD = DRIVE_PWM_D; analogWrite(PWMD, pwmD); motorDActive = true;
  restartSpeedControl();
}
void motorDStop() {
  digitalWrite(DIRD1, LOW); digitalWrite(DIRD2, LOW);
  analogWrite(PWMD, 0); motorDActive = false; restartSpeedControl();
}

#define MOTORA_FORWARD()  motorAForward()
#define MOTORA_BACKOFF()  motorABackoff()
#define MOTORA_STOP()     motorAStop()
#define MOTORB_FORWARD()  motorBForward()
#define MOTORB_BACKOFF()  motorBBackoff()
#define MOTORB_STOP()     motorBStop()
#define MOTORC_FORWARD()  motorCForward()
#define MOTORC_BACKOFF()  motorCBackoff()
#define MOTORC_STOP()     motorCStop()
#define MOTORD_FORWARD()  motorDForward()
#define MOTORD_BACKOFF()  motorDBackoff()
#define MOTORD_STOP()     motorDStop()

// ---------------- Motion primitives ---------------------
void advance()   { MOTORA_FORWARD(); MOTORB_FORWARD(); MOTORC_FORWARD(); MOTORD_FORWARD(); }
void back()      { MOTORA_BACKOFF(); MOTORB_BACKOFF(); MOTORC_BACKOFF(); MOTORD_BACKOFF(); }

void pivotLeft() { MOTORA_STOP();    MOTORB_FORWARD(); MOTORC_STOP();    MOTORD_FORWARD(); }
void pivotRight(){ MOTORA_FORWARD(); MOTORB_STOP();    MOTORC_FORWARD(); MOTORD_STOP();    }

void spinLeft()  { MOTORA_BACKOFF(); MOTORB_FORWARD(); MOTORC_BACKOFF(); MOTORD_FORWARD(); }
void spinRight() { MOTORA_FORWARD(); MOTORB_BACKOFF(); MOTORC_FORWARD(); MOTORD_BACKOFF(); }

void diagLeft()  { MOTORA_STOP();    MOTORB_STOP();    MOTORC_FORWARD(); MOTORD_BACKOFF(); }
void diagRight() { MOTORA_FORWARD(); MOTORB_BACKOFF(); MOTORC_STOP();    MOTORD_STOP();    }

void stopAll()   { MOTORA_STOP(); MOTORB_STOP(); MOTORC_STOP(); MOTORD_STOP(); }

// ---------------- Encoder ISRs --------------------------
void isrEncA() { encA += (digitalRead(ENC_A_A) == digitalRead(ENC_A_B)) ? -1 : +1; }
void isrEncB() { encB += (digitalRead(ENC_B_A) == digitalRead(ENC_B_B)) ? -1 : +1; }
void isrEncC() { encC += (digitalRead(ENC_C_A) == digitalRead(ENC_C_B)) ? -1 : +1; }
void isrEncD() { encD += (digitalRead(ENC_D_A) == digitalRead(ENC_D_B)) ? -1 : +1; }

void readNormalizedCounts(long &a, long &b, long &c, long &d) {
  noInterrupts();
  a = -encA;
  b = encB;
  c = -encC;
  d = encD;
  interrupts();
}

int adjustPwm(int pwm, long measuredCounts, int targetCounts) {
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
    lastControlA = a; lastControlB = b;
    lastControlC = c; lastControlD = d;
    speedControlInitialized = true;
    return;
  }

  long deltaA = labs(a - lastControlA);
  long deltaB = labs(b - lastControlB);
  long deltaC = labs(c - lastControlC);
  long deltaD = labs(d - lastControlD);

  int targetLeft  = TARGET_COUNTS_PER_INTERVAL;
  int targetRight = (int)(TARGET_COUNTS_PER_INTERVAL * (1.0f + STEER_TRIM));

  if (motorAActive) {
    pwmA = adjustPwm(pwmA, deltaA, targetLeft);
    analogWrite(PWMA, pwmA);
  }
  if (motorBActive) {
    pwmB = adjustPwm(pwmB, deltaB, targetRight);
    analogWrite(PWMB, pwmB);
  }
  if (motorCActive) {
    pwmC = adjustPwm(pwmC, deltaC, targetLeft);
    analogWrite(PWMC, pwmC);
  }
  if (motorDActive) {
    pwmD = adjustPwm(pwmD, deltaD, targetRight);
    analogWrite(PWMD, pwmD);
  }

  lastControlA = a; lastControlB = b;
  lastControlC = c; lastControlD = d;
}

// ---------------- ASCII Visualizer ----------------------
void printVisual(char A, char B, char C, char D) {
  Serial.println(F("     [FRONT]"));
  Serial.print  (F("    ("));
  Serial.print  (A);
  Serial.print  (F(")   ("));
  Serial.print  (B);
  Serial.println(F(")"));
  Serial.print  (F("    ("));
  Serial.print  (C);
  Serial.print  (F(")   ("));
  Serial.print  (D);
  Serial.println(F(")"));
  Serial.println(F("     [BACK]\r\n"));
}

// ---------------- Setup ---------------------------------
void setup() {
  Serial.begin(115200);
  
  pinMode(PWMA, OUTPUT); pinMode(DIRA1, OUTPUT); pinMode(DIRA2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(DIRB1, OUTPUT); pinMode(DIRB2, OUTPUT);
  pinMode(PWMC, OUTPUT); pinMode(DIRC1, OUTPUT); pinMode(DIRC2, OUTPUT);
  pinMode(PWMD, OUTPUT); pinMode(DIRD1, OUTPUT); pinMode(DIRD2, OUTPUT);

  stopAll();

  pinMode(ENC_A_A, INPUT_PULLUP); pinMode(ENC_A_B, INPUT_PULLUP);
  pinMode(ENC_B_A, INPUT_PULLUP); pinMode(ENC_B_B, INPUT_PULLUP);
  pinMode(ENC_C_A, INPUT_PULLUP); pinMode(ENC_C_B, INPUT_PULLUP);
  pinMode(ENC_D_A, INPUT_PULLUP); pinMode(ENC_D_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A_A), isrEncA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_A), isrEncB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_C_A), isrEncC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_D_A), isrEncD, CHANGE);

  Serial.println(F("\r\n=== READY ==="));
  Serial.println(F("Layout: A=FrontLeft, B=FrontRight, C=RearLeft, D=RearRight"));
  Serial.println(F("WASD/QEZC/JK = Movement | 1234/5678 = Individual Motors\r\n"));
}

// ---------------- Loop ----------------------------------
unsigned long lastReport = 0;

void loop() {
  // 1. Handle keyboard commands
  while (Serial.available() > 0) {
    char c = Serial.read();
    switch (c) {
      // --- Group Movements ---
      case 'w': advance();    Serial.println(F("Advance (All Fwd)")); printVisual('^', '^', '^', '^'); break;
      case 'x': back();       Serial.println(F("Back (All Rev)"));    printVisual('v', 'v', 'v', 'v'); break;
      
      case 'a': pivotLeft();  Serial.println(F("Left Pivot"));        printVisual('-', '^', '-', '^'); break;
      case 'd': pivotRight(); Serial.println(F("Right Pivot"));       printVisual('^', '-', '^', '-'); break;
      
      case 'q': diagLeft();   Serial.println(F("Front Left"));        printVisual('-', '-', '^', 'v'); break;
      case 'e': diagRight();  Serial.println(F("Front Right"));       printVisual('^', 'v', '-', '-'); break;
      
      case 'z': diagLeft();   Serial.println(F("Left Back"));         printVisual('-', '-', '^', 'v'); break; 
      case 'c': diagRight();  Serial.println(F("Right Back"));        printVisual('^', 'v', '-', '-'); break; 
      
      case 'j': spinLeft();   Serial.println(F("Rotate Left"));       printVisual('v', '^', 'v', '^'); break;
      case 'k': spinRight();  Serial.println(F("Rotate Right"));      printVisual('^', 'v', '^', 'v'); break;
      
      // --- Stops ---
      case 's': stopAll();    Serial.println(F("STOP ALL"));          printVisual('-', '-', '-', '-'); break;
      case 'g': stopAll();    Serial.println(F("STOP ALL"));          printVisual('-', '-', '-', '-'); break; 
      case '0': stopAll();    Serial.println(F("STOP ALL"));          printVisual('-', '-', '-', '-'); break; 
      
      // --- Individual Motors Forward ---
      case '1': MOTORA_FORWARD(); Serial.println(F("Motor A (FL) Fwd"));  printVisual('^', '-', '-', '-'); break;
      case '2': MOTORB_FORWARD(); Serial.println(F("Motor B (FR) Fwd"));  printVisual('-', '^', '-', '-'); break;
      case '3': MOTORC_FORWARD(); Serial.println(F("Motor C (RL) Fwd"));  printVisual('-', '-', '^', '-'); break;
      case '4': MOTORD_FORWARD(); Serial.println(F("Motor D (RR) Fwd"));  printVisual('-', '-', '-', '^'); break;
      
      // --- Individual Motors Backward ---
      case '5': MOTORA_BACKOFF(); Serial.println(F("Motor A (FL) Back")); printVisual('v', '-', '-', '-'); break;
      case '6': MOTORB_BACKOFF(); Serial.println(F("Motor B (FR) Back")); printVisual('-', 'v', '-', '-'); break;
      case '7': MOTORC_BACKOFF(); Serial.println(F("Motor C (RL) Back")); printVisual('-', '-', 'v', '-'); break;
      case '8': MOTORD_BACKOFF(); Serial.println(F("Motor D (RR) Back")); printVisual('-', '-', '-', 'v'); break;
      
      default: break; // Ignore newlines, spaces, etc.
    }
  }

  // 2. Adjust active motor PWMs to maintain equal encoder speed.
  updateSpeedControl();

  // 3. Report encoder counts every 2500 ms (2.5 seconds)
  if (millis() - lastReport >= 2500) {
    lastReport = millis();
    
    // Normalize encoder signs so forward wheel motion is positive.
    long a, b, c, d;
    readNormalizedCounts(a, b, c, d);

    // Print to PC
    Serial.print(F("ENC,"));
    Serial.print(a); Serial.print(',');
    Serial.print(b); Serial.print(',');
    Serial.print(c); Serial.print(',');
    Serial.println(d);
  }
}
