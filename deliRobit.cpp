#include <Arduino.h>

// ------------ USER CONFIG ------------
const float WHEEL_DIAMETER_IN   = 5.0;      // inches
const float TRACK_WIDTH_IN      = 8.0;      // distance between wheel contact patches (inches) -> adjust for your robot
const long  COUNTS_PER_WHEEL_REV = 980;     //
const int   BASE_PWM            = 150;      // baseline PWM (0..255) for motion
const int   MAX_PWM             = 255;      // clamp
const float STRAIGHT_KP         = 0.8;      // proportional gain for keeping straight (counts error -> PWM trim)
const float SPIN_KP             = 0.8;      // proportional gain for balancing counts while spinning
// ------------------------------------

// Motor driver pins (single DIR + PWM per side). Change if needed.
const int PWM_L = 9;
const int DIR_L = 8;
const int PWM_R = 10;
const int DIR_R = 7;

// Encoder pins (A uses interrupt)
const byte ENC_L_A = 2;  // INT0
const byte ENC_L_B = 4;
const byte ENC_R_A = 3;  // INT1
const byte ENC_R_B = 5;

// Encoder state
volatile long encCountL = 0;
volatile long encCountR = 0;
volatile byte lastLA = LOW, lastRA = LOW;

// Derived geometry
const float PI_F          = 3.14159265358979f;
const float WHEEL_CIRC_IN = PI_F * WHEEL_DIAMETER_IN;
const float COUNTS_PER_IN = (float)COUNTS_PER_WHEEL_REV / WHEEL_CIRC_IN;

// Utility clamp
int clampPWM(int v) {
  if (v < 0)   return 0;
  if (v > MAX_PWM) return MAX_PWM;
  return v;
}

// Low-level motor control: dir=true forward, false reverse
void setMotor(int pwmPin, int dirPin, int pwm, bool dir) {
  digitalWrite(dirPin, dir ? HIGH : LOW);
  analogWrite(pwmPin, clampPWM(pwm));
}

// Stop both
void stopMotors() {
  analogWrite(PWM_L, 0);
  analogWrite(PWM_R, 0);
}

// ISR helpers (fast quadrature step)
inline void updateEncoder(volatile long &count, byte pinA, byte pinB, volatile byte &lastA) {
  byte a = digitalRead(pinA);
  byte b = digitalRead(pinB);
  // Rising edge detect on A for direction decision; simple, clean, good enough
  if (lastA == LOW && a == HIGH) {
    if (b == LOW) count++; else count--;
  }
  lastA = a;
}

// Left encoder ISR
void IRAM_ATTR isrLeft() {
  updateEncoder(encCountL, ENC_L_A, ENC_L_B, lastLA);
}

// Right encoder ISR
void IRAM_ATTR isrRight() {
  updateEncoder(encCountR, ENC_R_A, ENC_R_B, lastRA);
}

// Reset encoders
void resetEncoders() {
  noInterrupts();
  encCountL = 0;
  encCountR = 0;
  interrupts();
}

// ----- High-level Moves -----
// Drive straight a signed distance (inches). Positive = forward, negative = reverse.
void driveStraightDistance(float inches, int basePwm = BASE_PWM, unsigned long timeoutMs = 10000UL) {
  long targetCounts = lround(fabs(inches) * COUNTS_PER_IN);
  bool forward = (inches >= 0);

  resetEncoders();

  unsigned long start = millis();
  while ((labs(encCountL) < targetCounts || labs(encCountR) < targetCounts) &&
         (millis() - start < timeoutMs)) {

    long eL = targetCounts - labs(encCountL);
    long eR = targetCounts - labs(encCountR);

    // Keep straight: trim based on count mismatch
    long mismatch = encCountL - encCountR; // >0 means left is ahead
    int trim = (int)(STRAIGHT_KP * (float)mismatch);

    int pwmL = clampPWM(basePwm - trim);
    int pwmR = clampPWM(basePwm + trim);

    setMotor(PWM_L, DIR_L, pwmL, forward);
    setMotor(PWM_R, DIR_R, pwmR, forward);

    // If one wheel reached target, let the other catch up
    if (labs(encCountL) >= targetCounts) analogWrite(PWM_L, 0);
    if (labs(encCountR) >= targetCounts) analogWrite(PWM_R, 0);
  }
  stopMotors();
}

// Drive straight for a duration (ms). Positive basePwm = forward, negative = reverse.
void driveStraightTime(int pwm, unsigned long ms) {
  bool forward = pwm >= 0;
  int base = abs(pwm);

  resetEncoders();
  unsigned long start = millis();
  while (millis() - start < ms) {
    long mismatch = encCountL - encCountR;
    int trim = (int)(STRAIGHT_KP * (float)mismatch);
    int pwmL = clampPWM(base - trim);
    int pwmR = clampPWM(base + trim);
    setMotor(PWM_L, DIR_L, pwmL, forward);
    setMotor(PWM_R, DIR_R, pwmR, forward);
  }
  stopMotors();
}

// Spin in place by a signed angle in degrees. Positive = spin CCW (left fwd, right rev), negative = CW.
void spinDegrees(float deg, int basePwm = BASE_PWM, unsigned long timeoutMs = 10000UL) {
  // In-place rotation: each wheel travels arc length = (theta_rad * track_width/2)
  float thetaRad = deg * PI_F / 180.0f;
  float arcIn    = fabs(thetaRad) * (TRACK_WIDTH_IN / 2.0f);
  long  targetCounts = lround(arcIn * COUNTS_PER_IN);

  bool ccw = (deg >= 0); // CCW: left forward, right reverse

  resetEncoders();

  unsigned long start = millis();
  while ((labs(encCountL) < targetCounts || labs(encCountR) < targetCounts) &&
         (millis() - start < timeoutMs)) {

    // Balance counts so both sides hit their arc at the same time
    long mismatch = encCountL - (-encCountR); // right is negative when reversing
    int trim = (int)(SPIN_KP * (float)mismatch);

    int pwmL = clampPWM(basePwm - trim);
    int pwmR = clampPWM(basePwm + trim);

    // CCW: L forward, R reverse; CW: L reverse, R forward
    setMotor(PWM_L, DIR_L, pwmL, ccw);
    setMotor(PWM_R, DIR_R, pwmR, !ccw);

    if (labs(encCountL) >= targetCounts) analogWrite(PWM_L, 0);
    if (labs(encCountR) >= targetCounts) analogWrite(PWM_R, 0);
  }
  stopMotors();
}

// ----- Arduino Lifecycle -----
void setup() {
  Serial.begin(57600);

  pinMode(PWM_L, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_R, OUTPUT);

  pinMode(ENC_L_A, INPUT);
  pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT);
  pinMode(ENC_R_B, INPUT);

  // Initialize last states
  lastLA = digitalRead(ENC_L_A);
  lastRA = digitalRead(ENC_R_A);

  // Attach interrupts on A channels
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRight, CHANGE);

  // Small hello
  Serial.println("Two-Wheel Encoder Drive Ready.");
  Serial.print("Counts/inch = "); Serial.println(COUNTS_PER_IN, 6);
}
