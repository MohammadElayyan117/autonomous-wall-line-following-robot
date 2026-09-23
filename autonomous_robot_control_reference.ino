/*
  Autonomous Wall & Line Following Robot with Obstacle Avoidance
  ----------------------------------------------------------------
  Reconstructed reference implementation based on the documented
  project behavior and the visible hardware setup.

  IMPORTANT:
  This is NOT the recovered original firmware. Pin assignments and
  sensor logic may need to be adjusted to match the exact robot wiring.

  Assumed hardware:
  - Arduino Uno
  - L298N motor driver
  - 2x DC motors
  - HC-SR04 ultrasonic sensor
  - SG90 servo for ultrasonic scanning
  - 2x digital IR line sensors for boundary detection
*/

#include <Servo.h>

// -------------------- Motor Driver (L298N) --------------------
const int ENA = 5;   // Left motor speed (PWM)
const int IN1 = 7;
const int IN2 = 8;

const int ENB = 6;   // Right motor speed (PWM)
const int IN3 = 9;
const int IN4 = 10;

// -------------------- Ultrasonic Sensor -----------------------
const int TRIG_PIN = 12;
const int ECHO_PIN = 11;

// -------------------- Servo ----------------------------------
const int SERVO_PIN = 3;
Servo scanner;

// -------------------- Line Sensors ----------------------------
// Adjust these pins if your sensors were connected elsewhere.
const int LEFT_LINE_SENSOR  = A0;
const int RIGHT_LINE_SENSOR = A1;

// Many IR line modules output LOW when the line is detected.
// Change to HIGH if your modules behave the opposite way.
const int LINE_DETECTED_STATE = LOW;

// -------------------- Navigation Parameters -------------------
const int BASE_SPEED = 150;
const int TURN_SPEED = 160;

const float OBSTACLE_DISTANCE_CM = 20.0;
const float TARGET_WALL_DISTANCE_CM = 15.0;
const float WALL_TOLERANCE_CM = 4.0;

const int FRONT_ANGLE = 90;
const int LEFT_ANGLE  = 155;
const int RIGHT_ANGLE = 25;

// -------------------------------------------------------------

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(LEFT_LINE_SENSOR, INPUT);
  pinMode(RIGHT_LINE_SENSOR, INPUT);

  scanner.attach(SERVO_PIN);
  scanner.write(FRONT_ANGLE);
  delay(500);

  stopMotors();

  Serial.println("Autonomous robot controller started.");
}

void loop() {
  bool leftBoundary  = digitalRead(LEFT_LINE_SENSOR)  == LINE_DETECTED_STATE;
  bool rightBoundary = digitalRead(RIGHT_LINE_SENSOR) == LINE_DETECTED_STATE;

  // Highest priority: keep the robot inside the path boundary.
  if (leftBoundary || rightBoundary) {
    handleBoundary(leftBoundary, rightBoundary);
    return;
  }

  float frontDistance = readDistanceAt(FRONT_ANGLE);

  Serial.print("Front: ");
  Serial.print(frontDistance);
  Serial.println(" cm");

  // Second priority: avoid an obstacle in front.
  if (frontDistance > 0 && frontDistance <= OBSTACLE_DISTANCE_CM) {
    avoidObstacle();
    return;
  }

  // Normal operation: follow the wall while moving inside the path.
  followWall();
}

// ==================== Navigation Logic ========================

void handleBoundary(bool leftDetected, bool rightDetected) {
  stopMotors();
  delay(80);

  // Back away from the boundary first.
  drive(-130, -130);
  delay(250);

  if (leftDetected && !rightDetected) {
    // Left boundary detected -> steer right.
    drive(TURN_SPEED, -TURN_SPEED);
    delay(300);
  }
  else if (rightDetected && !leftDetected) {
    // Right boundary detected -> steer left.
    drive(-TURN_SPEED, TURN_SPEED);
    delay(300);
  }
  else {
    // Both sensors detected a boundary -> turn around.
    drive(TURN_SPEED, -TURN_SPEED);
    delay(550);
  }

  stopMotors();
  delay(80);
}

void avoidObstacle() {
  stopMotors();
  delay(120);

  float leftDistance  = readDistanceAt(LEFT_ANGLE);
  float rightDistance = readDistanceAt(RIGHT_ANGLE);

  scanner.write(FRONT_ANGLE);
  delay(150);

  Serial.print("Left: ");
  Serial.print(leftDistance);
  Serial.print(" cm | Right: ");
  Serial.print(rightDistance);
  Serial.println(" cm");

  // Turn toward the side with more free space.
  if (leftDistance > rightDistance) {
    turnLeft();
  } else {
    turnRight();
  }

  stopMotors();
  delay(100);
}

void followWall() {
  // This implementation follows the wall on the LEFT side.
  float wallDistance = readDistanceAt(LEFT_ANGLE);

  scanner.write(FRONT_ANGLE);
  delay(80);

  if (wallDistance <= 0) {
    // Invalid reading: move slowly forward.
    drive(110, 110);
    return;
  }

  Serial.print("Wall distance: ");
  Serial.print(wallDistance);
  Serial.println(" cm");

  float error = wallDistance - TARGET_WALL_DISTANCE_CM;

  if (error > WALL_TOLERANCE_CM) {
    // Too far from wall -> steer toward the wall (left).
    drive(110, BASE_SPEED);
  }
  else if (error < -WALL_TOLERANCE_CM) {
    // Too close to wall -> steer away from wall (right).
    drive(BASE_SPEED, 110);
  }
  else {
    // Good distance -> continue forward.
    drive(BASE_SPEED, BASE_SPEED);
  }
}

void turnLeft() {
  drive(-TURN_SPEED, TURN_SPEED);
  delay(400);
}

void turnRight() {
  drive(TURN_SPEED, -TURN_SPEED);
  delay(400);
}

// ==================== Sensor Functions ========================

float readDistanceAt(int angle) {
  scanner.write(angle);
  delay(220);

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);

  if (duration == 0) {
    return -1.0;  // Timeout / invalid reading
  }

  float distanceCm = duration * 0.0343 / 2.0;

  if (distanceCm < 2.0 || distanceCm > 300.0) {
    return -1.0;
  }

  return distanceCm;
}

// ==================== Motor Functions =========================

// leftSpeed/rightSpeed range: -255 to 255
void drive(int leftSpeed, int rightSpeed) {
  setMotor(ENA, IN1, IN2, leftSpeed);
  setMotor(ENB, IN3, IN4, rightSpeed);
}

void setMotor(int enablePin, int inA, int inB, int speedValue) {
  speedValue = constrain(speedValue, -255, 255);

  if (speedValue > 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
  }
  else if (speedValue < 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
  }
  else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
  }

  analogWrite(enablePin, abs(speedValue));
}

void stopMotors() {
  drive(0, 0);
}
