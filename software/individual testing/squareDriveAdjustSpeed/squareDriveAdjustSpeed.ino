#include <Arduino.h>

// ============================================================
// Teensy 4.1 holonomic (X-drive) square test
// Motor wiring/pinout reused from the original soccer robot code.
// ============================================================

const int M1a = 4; // FRONT LEFT
const int M1b = 5;
const int M2a = 8; // FRONT RIGHT
const int M2b = 9;
const int M3a = 3; // BACK LEFT
const int M3b = 2;
const int M4a = 6; // BACK RIGHT
const int M4b = 7;

const int button1 = A6;
const int button2 = A7;
const int button3 = A8;

int state = 0;

// ---- Motor calibration multipliers ----
// 1.00 = no correction
// <1.00 = slower
// >1.00 = faster
const float motorMult[5] = {
  0.0f,   // unused
  1.00f,  // M1 Front Left
  1.00f,  // M2 Front Right
  0.9f,  // M3 Back Left
  1.00f   // M4 Back Right
};

// ---- Drive tuning ----
const float DRIVE_SPEED = 0.3;
const unsigned long SIDE_TIME_MS = 1000;
const unsigned long PAUSE_TIME_MS = 200;

void SetSpeed(int motor, int pwm);
void drive(float direction_deg, float speed, float rotation);
void stopAllMotors();
void driveSide(float direction_deg);

void setup() {
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

  Serial.begin(115200);
  delay(100);
  Serial.println("Teensy 4.1 holonomic square-drive demo starting...");
}

void loop() {
  if (digitalRead(button1) == HIGH) {
    state = 1;
    delay(200);
  }

  if (state == 1) {
    driveSide(180);
    driveSide(270);
    driveSide(0);
    driveSide(90);
  }

  stopAllMotors();
  state = 0;
}

// Drive in one direction for SIDE_TIME_MS, then stop and pause.
void driveSide(float direction_deg) {
  drive(direction_deg, DRIVE_SPEED, 0.0);
  delay(SIDE_TIME_MS);
  stopAllMotors();
  delay(PAUSE_TIME_MS);
}

void stopAllMotors() {
  SetSpeed(1, 0);
  SetSpeed(2, 0);
  SetSpeed(3, 0);
  SetSpeed(4, 0);
}

// X-drive (holonomic) kinematics
// direction_deg: 0 = forward, 90 = right, 180 = backward, 270 = left
void drive(float direction_deg, float speed, float rotation) {
  speed = constrain(speed, 0.0f, 1.0f);

  float direction_rad = direction_deg * PI / 180.0f;
  float vx = speed * cos(direction_rad);
  float vy = speed * sin(direction_rad);

  float wheel_speeds[4];

  wheel_speeds[0] = vx * sin(45 * PI / 180.0f)
                  + vy * cos(45 * PI / 180.0f)
                  + rotation; // FL

  wheel_speeds[1] = vx * sin(-45 * PI / 180.0f)
                  + vy * cos(-45 * PI / 180.0f)
                  + rotation; // FR

  wheel_speeds[2] = vx * sin(-135 * PI / 180.0f)
                  + vy * cos(-135 * PI / 180.0f)
                  + rotation; // BR

  wheel_speeds[3] = vx * sin(135 * PI / 180.0f)
                  + vy * cos(135 * PI / 180.0f)
                  + rotation; // BL

  // Normalize wheel speeds
  float max_speed = 0.0f;

  for (int i = 0; i < 4; i++) {
    if (abs(wheel_speeds[i]) > max_speed) {
      max_speed = abs(wheel_speeds[i]);
    }
  }

  if (max_speed > 1.0f) {
    for (int i = 0; i < 4; i++) {
      wheel_speeds[i] /= max_speed;
    }
  }

  SetSpeed(1, (int)(wheel_speeds[0] * 255)); // FL
  SetSpeed(2, (int)(wheel_speeds[1] * 255)); // FR
  SetSpeed(3, (int)(wheel_speeds[3] * 255)); // BL
  SetSpeed(4, (int)(wheel_speeds[2] * 255)); // BR
}

void SetSpeed(int motor, int pwm) {

  // Apply motor calibration multiplier
  pwm = (int)(pwm * motorMult[motor]);

  pwm = constrain(pwm, -255, 255);

  int pinA, pinB;

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
  }
  else if (pwm < 0) {
    analogWrite(pinA, 0);
    analogWrite(pinB, -pwm);
  }
  else {
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
  }
}