/*
  Autonomous Wall & Line Following Robot with Obstacle Avoidance
  ==============================================================
  Arduino control program implementing the documented project behavior:

  - Stay inside a defined path boundary.
  - Maintain a safe distance from the wall.
  - Detect obstacles in front.
  - Turn around / bypass the obstacle.
  - Continue moving forward inside the allowed path.

  IMPORTANT HARDWARE NOTE
  -----------------------
  The original wiring/pin map was not available, so the pin assignments,
  speeds, angles, and distance thresholds below are clean implementation
  choices and can be changed in one place.

  Assumed hardware:
  - Arduino Uno
  - L298N motor driver
  - 2 DC motors
  - HC-SR04 ultrasonic sensor mounted on an SG90 servo
  - 2 digital IR line/boundary sensors

  Documented path width: approximately 50 cm.
*/

#include <Servo.h>

// ============================================================
// USER CONFIGURATION
// ============================================================

// ---------- L298N motor driver ----------
const uint8_t LEFT_EN  = 5;   // PWM
const uint8_t LEFT_IN1 = 7;
const uint8_t LEFT_IN2 = 8;

const uint8_t RIGHT_EN  = 6;  // PWM
const uint8_t RIGHT_IN1 = 9;
const uint8_t RIGHT_IN2 = 10;

// If one motor spins in the wrong direction, change its flag.
const bool LEFT_MOTOR_REVERSED  = false;
const bool RIGHT_MOTOR_REVERSED = false;

// ---------- Ultrasonic sensor ----------
const uint8_t TRIG_PIN = 12;
const uint8_t ECHO_PIN = 11;

// ---------- Servo ----------
const uint8_t SERVO_PIN = 3;
Servo ultrasonicServo;

// ---------- Boundary / line sensors ----------
const uint8_t LEFT_LINE_PIN  = A0;
const uint8_t RIGHT_LINE_PIN = A1;

// Most IR line modules output LOW when they detect a dark line.
// Change to HIGH if your modules work the opposite way.
const uint8_t LINE_DETECTED_STATE = LOW;

// ---------- Servo scan angles ----------
const uint8_t FRONT_ANGLE = 90;
const uint8_t LEFT_ANGLE  = 150;
const uint8_t RIGHT_ANGLE = 30;

// ---------- Navigation ----------
const float TARGET_WALL_DISTANCE_CM = 15.0;
const float WALL_TOLERANCE_CM       = 3.0;
const float OBSTACLE_DISTANCE_CM    = 20.0;

// Motor PWM values: 0..255
const int BASE_SPEED       = 150;
const int CORRECTION_SPEED = 105;
const int TURN_SPEED       = 165;
const int REVERSE_SPEED    = 135;

// Maneuver timing
const unsigned long REVERSE_TIME_MS       = 250;
const unsigned long BOUNDARY_TURN_MS      = 300;
const unsigned long UTURN_TIME_MS         = 560;
const unsigned long OBSTACLE_TURN_MS      = 420;
const unsigned long OBSTACLE_FORWARD_MS   = 500;
const unsigned long OBSTACLE_RECENTER_MS  = 330;

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(9600);

  pinMode(LEFT_EN, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

  pinMode(RIGHT_EN, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(LEFT_LINE_PIN, INPUT);
  pinMode(RIGHT_LINE_PIN, INPUT);

  ultrasonicServo.attach(SERVO_PIN);
  ultrasonicServo.write(FRONT_ANGLE);
  delay(500);

  stopMotors();

  Serial.println();
  Serial.println(F("Autonomous robot controller started."));
  Serial.println(F("Modes: boundary tracking + wall following + obstacle avoidance"));
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  const bool leftBoundaryDetected =
      digitalRead(LEFT_LINE_PIN) == LINE_DETECTED_STATE;

  const bool rightBoundaryDetected =
      digitalRead(RIGHT_LINE_PIN) == LINE_DETECTED_STATE;

  // Priority 1: never leave the allowed path.
  if (leftBoundaryDetected || rightBoundaryDetected) {
    handleBoundary(leftBoundaryDetected, rightBoundaryDetected);
    return;
  }

  // Priority 2: detect a front obstacle.
  const float frontDistance = readDistanceCmAt(FRONT_ANGLE);

  printDistance(F("Front"), frontDistance);

  if (isValidDistance(frontDistance) &&
      frontDistance <= OBSTACLE_DISTANCE_CM) {
    avoidObstacle();
    return;
  }

  // Priority 3: normal wall following.
  followWall();
}

// ============================================================
// NAVIGATION
// ============================================================

void handleBoundary(bool leftDetected, bool rightDetected) {
  Serial.println(F("[BOUNDARY] Path edge detected"));

  stopMotors();
  delay(80);

  // Move away from the line first.
  drive(-REVERSE_SPEED, -REVERSE_SPEED);
  delay(REVERSE_TIME_MS);

  if (leftDetected && !rightDetected) {
    // Left edge -> steer right.
    Serial.println(F("[BOUNDARY] Left edge -> steering right"));
    pivotRight(TURN_SPEED);
    delay(BOUNDARY_TURN_MS);
  }
  else if (rightDetected && !leftDetected) {
    // Right edge -> steer left.
    Serial.println(F("[BOUNDARY] Right edge -> steering left"));
    pivotLeft(TURN_SPEED);
    delay(BOUNDARY_TURN_MS);
  }
  else {
    // Both sensors reached the boundary.
    Serial.println(F("[BOUNDARY] Both edges -> U-turn"));
    pivotRight(TURN_SPEED);
    delay(UTURN_TIME_MS);
  }

  stopMotors();
  delay(80);
}

void avoidObstacle() {
  Serial.println(F("[OBSTACLE] Obstacle ahead"));

  stopMotors();
  delay(120);

  // Scan both sides.
  const float leftClearance  = readDistanceCmAt(LEFT_ANGLE);
  const float rightClearance = readDistanceCmAt(RIGHT_ANGLE);

  printDistance(F("Left clearance"), leftClearance);
  printDistance(F("Right clearance"), rightClearance);

  ultrasonicServo.write(FRONT_ANGLE);
  delay(120);

  // Pick the side with more free space.
  const bool goLeft = chooseLeftSide(leftClearance, rightClearance);

  if (goLeft) {
    Serial.println(F("[OBSTACLE] Bypassing on LEFT"));

    pivotLeft(TURN_SPEED);
    delay(OBSTACLE_TURN_MS);

    drive(BASE_SPEED, BASE_SPEED);
    delay(OBSTACLE_FORWARD_MS);

    pivotRight(TURN_SPEED);
    delay(OBSTACLE_RECENTER_MS);
  }
  else {
    Serial.println(F("[OBSTACLE] Bypassing on RIGHT"));

    pivotRight(TURN_SPEED);
    delay(OBSTACLE_TURN_MS);

    drive(BASE_SPEED, BASE_SPEED);
    delay(OBSTACLE_FORWARD_MS);

    pivotLeft(TURN_SPEED);
    delay(OBSTACLE_RECENTER_MS);
  }

  stopMotors();
  delay(100);
}

void followWall() {
  // The implementation follows the wall on the LEFT side.
  const float wallDistance = readDistanceCmAt(LEFT_ANGLE);

  printDistance(F("Wall"), wallDistance);

  ultrasonicServo.write(FRONT_ANGLE);
  delay(70);

  if (!isValidDistance(wallDistance)) {
    // If the reading is temporarily invalid, keep moving slowly.
    Serial.println(F("[WALL] Invalid reading -> slow forward"));
    drive(CORRECTION_SPEED, CORRECTION_SPEED);
    return;
  }

  const float error = wallDistance - TARGET_WALL_DISTANCE_CM;

  if (error > WALL_TOLERANCE_CM) {
    // Too far from wall -> steer toward it.
    Serial.println(F("[WALL] Too far -> steer left"));
    drive(CORRECTION_SPEED, BASE_SPEED);
  }
  else if (error < -WALL_TOLERANCE_CM) {
    // Too close -> steer away from wall.
    Serial.println(F("[WALL] Too close -> steer right"));
    drive(BASE_SPEED, CORRECTION_SPEED);
  }
  else {
    Serial.println(F("[WALL] Distance OK -> forward"));
    drive(BASE_SPEED, BASE_SPEED);
  }
}

// ============================================================
// ULTRASONIC SENSOR
// ============================================================

float readDistanceCmAt(uint8_t angle) {
  ultrasonicServo.write(angle);
  delay(180);

  // Average valid samples to reduce noisy readings.
  float total = 0.0;
  uint8_t validSamples = 0;

  for (uint8_t i = 0; i < 3; i++) {
    const float d = readSingleDistanceCm();

    if (isValidDistance(d)) {
      total += d;
      validSamples++;
    }

    delay(25);
  }

  if (validSamples == 0) {
    return -1.0;
  }

  return total / validSamples;
}

float readSingleDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  const unsigned long duration =
      pulseIn(ECHO_PIN, HIGH, 30000UL);

  if (duration == 0) {
    return -1.0;
  }

  const float distanceCm = duration * 0.0343f / 2.0f;

  if (distanceCm < 2.0 || distanceCm > 300.0) {
    return -1.0;
  }

  return distanceCm;
}

bool isValidDistance(float distanceCm) {
  return distanceCm >= 2.0 && distanceCm <= 300.0;
}

bool chooseLeftSide(float leftDistance, float rightDistance) {
  const bool leftValid  = isValidDistance(leftDistance);
  const bool rightValid = isValidDistance(rightDistance);

  if (leftValid && rightValid) {
    return leftDistance >= rightDistance;
  }

  if (leftValid) {
    return true;
  }

  if (rightValid) {
    return false;
  }

  // Safe default if both scans fail.
  return false;
}

// ============================================================
// MOTOR CONTROL
// ============================================================

void drive(int leftSpeed, int rightSpeed) {
  setMotor(
    LEFT_EN,
    LEFT_IN1,
    LEFT_IN2,
    leftSpeed,
    LEFT_MOTOR_REVERSED
  );

  setMotor(
    RIGHT_EN,
    RIGHT_IN1,
    RIGHT_IN2,
    rightSpeed,
    RIGHT_MOTOR_REVERSED
  );
}

void setMotor(
  uint8_t enablePin,
  uint8_t in1,
  uint8_t in2,
  int speedValue,
  bool reversed
) {
  speedValue = constrain(speedValue, -255, 255);

  if (reversed) {
    speedValue = -speedValue;
  }

  if (speedValue > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (speedValue < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  analogWrite(enablePin, abs(speedValue));
}

void pivotLeft(int speedValue) {
  drive(-speedValue, speedValue);
}

void pivotRight(int speedValue) {
  drive(speedValue, -speedValue);
}

void stopMotors() {
  drive(0, 0);
}

// ============================================================
// DEBUG
// ============================================================

void printDistance(const __FlashStringHelper* label, float value) {
  Serial.print(label);
  Serial.print(F(": "));

  if (isValidDistance(value)) {
    Serial.print(value, 1);
    Serial.println(F(" cm"));
  }
  else {
    Serial.println(F("invalid"));
  }
}
