#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// ============================================================
// BNO08X IMU
// ============================================================

#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// ============================================================
// Motor Pins
// ============================================================

const int M1a = 4;  // FRONT LEFT
const int M1b = 5;

const int M2a = 8;  // FRONT RIGHT
const int M2b = 9;

const int M3a = 3;  // BACK LEFT
const int M3b = 2;

const int M4a = 6;  // BACK RIGHT
const int M4b = 7;

// ============================================================
// Buttons
// ============================================================

const int button1 = A6;
const int button2 = A7;
const int button3 = A8;

// ============================================================
// Motor Calibration
// ============================================================

// 1.00 = nominal
// >1.00 = increase motor speed
// <1.00 = decrease motor speed

const float motorMult[5] = {
  1.0f,   // unused
  1.00f,  // M1 FL
  1.00f,  // M2 FR
  1.00f,  // M3 BL
  1.00f   // M4 BR
};

// ============================================================
// Drive Settings
// ============================================================

const float DRIVE_SPEED = 0.30f;

const unsigned long SIDE_TIME_MS = 1000;
const unsigned long PAUSE_TIME_MS = 200;

// ============================================================
// Heading Hold Settings (PID)
// ============================================================

// Tuning guide:
// - HEADING_KP: main correction strength. Too high = overshoot/oscillation.
// - HEADING_KD: damps overshoot caused by KP. Raise this first if the robot
//               is overcorrecting / oscillating around the target heading.
// - HEADING_KI: eliminates slow steady-state drift that KP+KD alone can't
//               remove. Leave at 0 unless you see a persistent residual
//               offset that never converges. Raise in small steps (0.0005).
//
// Start here, then tune in this order: KP -> KD -> KI.

const float HEADING_KP = 0.005f;
const float HEADING_KI = 0.0f;
const float HEADING_KD = 0.0001f;

// Clamp on the accumulated integral term (anti-windup), expressed in the
// same units as the correction output (-1.0 .. 1.0 motor scale).
const float HEADING_INTEGRAL_MAX = 0.20f;

float currentYawDeg = 0.0f;
float desiredHeadingDeg = 0.0f;

// PID internal state
float headingIntegral = 0.0f;
float headingLastError = 0.0f;
unsigned long headingLastTimeMs = 0;
bool headingPidInitialized = false;

// ============================================================
// Function Prototypes
// ============================================================

void SetSpeed(int motor, int pwm);
void stopAllMotors();

void drive(
  float direction_deg,
  float speed,
  float rotation);

void driveSide(float direction_deg);

void setReports();
void updateIMU();

float quaternionToYawDegrees(
  float real,
  float i,
  float j,
  float k);

float angleError(
  float target,
  float current);

float headingCorrection();
void resetHeadingPID();

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(115200);

  pinMode(M1a, OUTPUT);
  pinMode(M1b, OUTPUT);

  pinMode(M2a, OUTPUT);
  pinMode(M2b, OUTPUT);

  pinMode(M3a, OUTPUT);
  pinMode(M3b, OUTPUT);

  pinMode(M4a, OUTPUT);
  pinMode(M4b, OUTPUT);

  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(button3, INPUT);

  delay(100);

  Serial.println("Starting...");

  if (!bno08x.begin_I2C()) {
    Serial.println("BNO08x not found!");

    while (1) {
      delay(10);
    }
  }

  Serial.println("BNO08x Found");

  setReports();

  delay(500);

  updateIMU();

  Serial.print("Initial Heading: ");
  Serial.println(currentYawDeg);
}

// ============================================================
// Main Loop
// ============================================================

void loop() {
  updateIMU();

  if (digitalRead(button1) == HIGH) {
    delay(100);

    updateIMU();

    desiredHeadingDeg = currentYawDeg;

    Serial.print("Locked Heading: ");
    Serial.println(desiredHeadingDeg);

    driveSide(180);
    driveSide(270);
    driveSide(0);
    driveSide(90);

    stopAllMotors();
  }
}

// ============================================================
// BNO08X Setup
// ============================================================

void setReports() {
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable rotation vector");
  }
}

// ============================================================
// Quaternion -> Yaw
// ============================================================

float quaternionToYawDegrees(
  float real,
  float i,
  float j,
  float k) {
  float yaw =
    atan2(
      2.0f * (real * k + i * j),
      1.0f - 2.0f * (j * j + k * k));

  return yaw * 180.0f / PI;
}

// ============================================================
// IMU Update
// ============================================================

void updateIMU() {
  if (bno08x.wasReset()) {
    setReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
    currentYawDeg =
      quaternionToYawDegrees(
        sensorValue.un.gameRotationVector.real,
        sensorValue.un.gameRotationVector.i,
        sensorValue.un.gameRotationVector.j,
        sensorValue.un.gameRotationVector.k);
  }
}

// ============================================================
// Heading Helpers
// ============================================================

float angleError(
  float target,
  float current) {
  float error = target - current;

  while (error > 180.0f)
    error -= 360.0f;

  while (error < -180.0f)
    error += 360.0f;

  return error;
}

// Clears all PID state (integral, last error, timer). Call this any time
// a fresh heading-hold segment starts, so old accumulated error from a
// previous segment (or the pause between segments) doesn't bleed in and
// cause an overcorrection spike at the start of the next side.
void resetHeadingPID() {
  headingIntegral = 0.0f;
  headingLastError = 0.0f;
  headingLastTimeMs = millis();
  headingPidInitialized = false;
}

float headingCorrection() {
  unsigned long now = millis();

  float error =
    angleError(
      desiredHeadingDeg,
      currentYawDeg);

  float dt = 0.0f;

  if (headingPidInitialized) {
    dt = (now - headingLastTimeMs) / 1000.0f;
  }

  // Integral term, with anti-windup clamping.
  if (dt > 0.0f) {
    headingIntegral += error * dt;

    headingIntegral =
      constrain(
        headingIntegral,
        -HEADING_INTEGRAL_MAX,
        HEADING_INTEGRAL_MAX);
  }

  // Derivative term. Skipped on the very first call (no valid dt yet),
  // which avoids a derivative spike at the start of each segment.
  float derivative = 0.0f;

  if (dt > 0.0f) {
    derivative = (error - headingLastError) / dt;
  }

  float correction =
    (error * HEADING_KP) +
    (headingIntegral * HEADING_KI) +
    (derivative * HEADING_KD);

  correction =
    constrain(
      correction,
      -0.40f,
      0.40f);

  headingLastError = error;
  headingLastTimeMs = now;
  headingPidInitialized = true;

  return correction;
}

// ============================================================
// Drive One Side
// ============================================================

void driveSide(float direction_deg) {
  // Fresh PID state for this segment so the pause/stop time between
  // sides doesn't get folded into the integral or derivative terms.
  resetHeadingPID();

  unsigned long startTime = millis();

  while ((millis() - startTime) < SIDE_TIME_MS) {
    updateIMU();

    float rotation =
      headingCorrection();

    drive(
      direction_deg,
      DRIVE_SPEED,
      rotation);

    delay(5);
  }

  stopAllMotors();

  delay(PAUSE_TIME_MS);
}

// ============================================================
// Holonomic Drive
// ============================================================

void drive(
  float direction_deg,
  float speed,
  float rotation) {
  speed = constrain(speed, 0.0f, 1.0f);

  float direction_rad =
    direction_deg * PI / 180.0f;

  float vx =
    speed * cos(direction_rad);

  float vy =
    speed * sin(direction_rad);

  float wheel_speeds[4];

  wheel_speeds[0] =
    vx * sin(45 * PI / 180.0f) + vy * cos(45 * PI / 180.0f) + rotation;

  wheel_speeds[1] =
    vx * sin(-45 * PI / 180.0f) + vy * cos(-45 * PI / 180.0f) + rotation;

  wheel_speeds[2] =
    vx * sin(-135 * PI / 180.0f) + vy * cos(-135 * PI / 180.0f) + rotation;

  wheel_speeds[3] =
    vx * sin(135 * PI / 180.0f) + vy * cos(135 * PI / 180.0f) + rotation;

  float max_speed = 0.0f;

  for (int i = 0; i < 4; i++) {
    if (fabs(wheel_speeds[i]) > max_speed) {
      max_speed = fabs(wheel_speeds[i]);
    }
  }

  if (max_speed > 1.0f) {
    for (int i = 0; i < 4; i++) {
      wheel_speeds[i] /= max_speed;
    }
  }

  SetSpeed(1, wheel_speeds[0] * 255);
  SetSpeed(2, wheel_speeds[1] * 255);
  SetSpeed(3, wheel_speeds[3] * 255);
  SetSpeed(4, wheel_speeds[2] * 255);
}

// ============================================================
// Stop Motors
// ============================================================

void stopAllMotors() {
  SetSpeed(1, 0);
  SetSpeed(2, 0);
  SetSpeed(3, 0);
  SetSpeed(4, 0);
}

// ============================================================
// Motor Output
// ============================================================

void SetSpeed(int motor, int pwm) {
  pwm = (int)(pwm * motorMult[motor]);

  pwm = constrain(pwm, -255, 255);

  int pinA;
  int pinB;

  switch (motor) {
    case 1:
      pinA = M1a;
      pinB = M1b;
      break;

    case 2:
      pinA = M2a;
      pinB = M2b;
      break;

    case 3:
      pinA = M3a;
      pinB = M3b;
      break;

    case 4:
      pinA = M4a;
      pinB = M4b;
      break;

    default:
      return;
  }

  if (pwm > 0) {
    analogWrite(pinA, pwm);
    analogWrite(pinB, 0);
  } else if (pwm < 0) {
    analogWrite(pinA, 0);
    analogWrite(pinB, -pwm);
  } else {
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
  }
}