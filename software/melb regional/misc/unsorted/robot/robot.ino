#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

const int M1a = 2; // FRONT LEFT
const int M1b = 3;
const int M2a = 5; // FRONT RIGHT
const int M2b = 4;
const int M3a = 9; // BACK LEFT
const int M3b = 8;
const int M4a = 7; // BACK RIGHT
const int M4b = 6;

const int button1 = A6;
const int button2 = A7;
const int button3 = A8;

const float motorMult[5] = {
  1.0f,   // unused
  1.00f,  // M1 FL
  1.00f,  // M2 FR
  1.00f,  // M3 BL
  1.00f   // M4 BR
};

const float ACCEL_LIMIT = 1.1f;

// Hard ceiling on how fast the heading SETPOINT (desiredHeadingDeg) is
// allowed to ramp, in degrees/sec^2. This is analogous to ACCEL_LIMIT
// above but for rotation instead of translation speed.
//
// Previously desiredHeadingDeg was slid linearly from startYaw to
// targetRotation_deg over duration_ms - that's a CONSTANT angular
// velocity command, which means the setpoint's angular velocity jumps
// instantly from 0 to full speed at t=0 and instantly back to 0 at
// t=duration_ms. The heading PID would try to track that instant jump,
// causing a kick at the start/end of every rotating move.
//
// Instead, move() now builds a trapezoidal ANGLE profile (ramp up,
// optional cruise, ramp down) for desiredHeadingDeg itself, the same
// way the speed profile is built for translation, so the setpoint's
// angular velocity is continuous and bounded by this limit.
//
// Unlike ACCEL_LIMIT (which is allowed to reduce peak speed when time
// is short), the rotation profile has a fixed angle it MUST cover by
// the end of duration_ms (targetRotation_deg). If ROTATION_ACCEL_LIMIT
// is too low to cover the required rotation in the given duration even
// with a pure triangular (no-cruise) profile, computeRotationProfile()
// will widen the effective accel just enough to still land exactly on
// targetRotation_deg on time, rather than silently finishing the move
// with the wrong heading. Tune this value up if you see that happening
// often (it prints nothing by default - enable DEBUG_MOVE and watch for
// large rotations paired with short durations).

const float ROTATION_ACCEL_LIMIT = 720.0f; // deg/s^2

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
// Tune in this order: KP -> KD -> KI.

const float HEADING_KP = 0.005f;
const float HEADING_KI = 0.0f;
const float HEADING_KD = 0.001f;

// Clamp on the accumulated integral term (anti-windup), expressed in the
// same units as the correction output (-1.0 .. 1.0 motor scale).
const float HEADING_INTEGRAL_MAX = 0.20f;

// Set to +1.0 or -1.0 depending on which way your IMU's yaw increases.
// TEST: hold the robot still and rotate it BY HAND counter-clockwise as
// seen from directly above (bird's-eye view). Watch currentYawDeg over
// Serial. If it INCREASES, leave this at +1.0. If it DECREASES, change
// this to -1.0. Getting this backwards is the single most common cause
// of a holonomic robot arcing instead of going straight while rotating.
const float YAW_SIGN = 1.0f;

float currentYawDeg = 0.0f;
float desiredHeadingDeg = 0.0f;

// PID internal state
float headingIntegral = 0.0f;
float headingLastError = 0.0f;
unsigned long headingLastTimeMs = 0;
bool headingPidInitialized = false;

// ============================================================
// Move Profile (trapezoidal speed ramp)
// ============================================================

struct MoveProfile {
  float accelTime;   // seconds spent ramping speed up
  float cruiseTime;  // seconds spent holding peakSpeed (0 if triangular)
  float decelTime;   // seconds spent ramping speed down
  float peakSpeed;   // highest speed actually reached (<= requested maxSpeed)
};

// ============================================================
// Rotation Profile (trapezoidal angle ramp)
// ============================================================

// Unlike MoveProfile (which ramps a free-running speed for a duration
// with no fixed distance requirement), a rotation MUST land exactly on
// targetRotation_deg by the end of the move. So this profile is solved
// for a fixed total angle (rotationDelta) covered over totalTime_sec,
// rather than for a capped peak speed.
struct RotationProfile {
  float accelTime;        // seconds spent ramping angular velocity up
  float cruiseTime;       // seconds spent holding peakOmega (0 if triangular)
  float decelTime;        // seconds spent ramping angular velocity down
  float peakOmegaMag;     // magnitude of peak angular velocity, deg/s
  float effectiveAccelMag; // magnitude of accel actually used, deg/s^2
                            // (may exceed ROTATION_ACCEL_LIMIT if the
                            // requested rotation/time combo demanded it)
  float totalDelta;       // signed total rotation this profile covers, deg
};

// ============================================================
// Function Prototypes
// ============================================================

void SetSpeed(int motor, int pwm);
void stopAllMotors();

void drive(
  float direction_deg,
  float speed,
  float rotation);

void move(
  float direction_deg,
  unsigned long duration_ms,
  float targetRotation_deg,
  float maxSpeed);

MoveProfile computeMoveProfile(
  float maxSpeed,
  float totalTime_sec,
  float accelLimit);

float speedAtTime(
  const MoveProfile &profile,
  float t_sec,
  float totalTime_sec,
  float accelLimit);

RotationProfile computeRotationProfile(
  float rotationDelta_deg,
  float totalTime_sec,
  float accelLimit_degs2);

float angleAtTime(
  const RotationProfile &profile,
  float t_sec,
  float totalTime_sec);

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

    float startHeading = currentYawDeg;

    Serial.print("Starting Heading: ");
    Serial.println(startHeading);

    // Example sequence: a 1.5s-per-side square, each side also gradually
    // rotating a quarter turn, unwinding back to the start heading on the
    // final leg.
    //
    // direction_deg is FIELD-relative (see drive()), so each side's
    // direction is offset by startHeading - this anchors "field forward"
    // to whichever way the robot happened to be facing at button press,
    // the same reference the rotation targets below use. The robot will
    // trace a straight-sided square in that fixed frame even though it's
    // spinning a quarter turn on every side.
    move(startHeading + 0,   1500, startHeading + 90.0f,  0.7f);
    move(startHeading + 90,  1500, startHeading + 180.0f, 0.7f);
    move(startHeading + 180, 1500, startHeading + 270.0f, 0.7f);
    move(startHeading + 270, 1500, startHeading,          0.7f);

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
// a fresh heading-hold segment starts (i.e. at the start of every move()),
// so old accumulated error from a previous move doesn't bleed in and cause
// an overcorrection spike at the start of the next one.
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
  // which avoids a derivative spike at the start of each move.
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
// Move Profile: trapezoidal speed ramp solver
// ============================================================

// Works out how a move's speed should ramp up, cruise, and ramp down so
// the whole thing fits exactly into totalTime_sec without exceeding
// accelLimit. Falls back to a triangular (no cruise) profile if there
// isn't enough time to reach maxSpeed at the given accel limit.
MoveProfile computeMoveProfile(
  float maxSpeed,
  float totalTime_sec,
  float accelLimit) {

  MoveProfile profile;

  maxSpeed = constrain(maxSpeed, 0.0f, 1.0f);

  if (accelLimit <= 0.0001f || totalTime_sec <= 0.0001f) {
    // No meaningful ramp possible - just hold maxSpeed for the duration.
    profile.accelTime = 0.0f;
    profile.decelTime = 0.0f;
    profile.cruiseTime = totalTime_sec;
    profile.peakSpeed = maxSpeed;
    return profile;
  }

  float accelTimeFull = maxSpeed / accelLimit;

  if ((accelTimeFull * 2.0f) <= totalTime_sec) {
    // Trapezoid: enough time to ramp up, cruise, and ramp back down.
    profile.accelTime = accelTimeFull;
    profile.decelTime = accelTimeFull;
    profile.cruiseTime = totalTime_sec - (accelTimeFull * 2.0f);
    profile.peakSpeed = maxSpeed;
  } else {
    // Triangle: not enough time to reach maxSpeed at this accel limit.
    // Split the available time evenly between accel and decel instead,
    // so the move still respects accelLimit and still completes exactly
    // on time, just with a lower peak speed.
    profile.accelTime = totalTime_sec / 2.0f;
    profile.decelTime = totalTime_sec / 2.0f;
    profile.cruiseTime = 0.0f;
    profile.peakSpeed = accelLimit * profile.accelTime;
  }

  return profile;
}

// Returns the commanded speed (0.0-1.0) at a given elapsed time into the
// move, following the precomputed trapezoidal/triangular profile.
float speedAtTime(
  const MoveProfile &profile,
  float t_sec,
  float totalTime_sec,
  float accelLimit) {

  if (t_sec >= totalTime_sec) {
    return 0.0f;
  }

  if (t_sec < profile.accelTime) {
    return accelLimit * t_sec;
  }

  float cruiseEnd = profile.accelTime + profile.cruiseTime;

  if (t_sec < cruiseEnd) {
    return profile.peakSpeed;
  }

  float tIntoDecel = t_sec - cruiseEnd;
  float speed = profile.peakSpeed - (accelLimit * tIntoDecel);

  return constrain(speed, 0.0f, profile.peakSpeed);
}

// ============================================================
// Rotation Profile: trapezoidal angle ramp solver
// ============================================================

// Solves for the peak angular velocity (and accel/cruise/decel timing)
// of a trapezoidal profile that covers exactly rotationDelta_deg over
// totalTime_sec, without exceeding accelLimit_degs2.
//
// This is a fixed-distance trapezoidal motion profile (unlike
// computeMoveProfile, which is a fixed-time/capped-peak profile with no
// distance requirement). For a trapezoid with peak angular velocity V,
// ramp time t_a = V / accelLimit, the angle covered works out to:
//   D = V * (T - t_a) = V*T - V^2/accelLimit
// Solving that quadratic for V and taking the smaller root gives the
// minimum peak velocity that still covers D in time T while respecting
// accelLimit (the smaller root is the trapezoid/triangle solution; the
// larger root is a spurious high-speed alternative and is discarded).
//
// If even a pure triangular (no-cruise) profile can't cover D in time T
// at accelLimit, the required accel is widened just enough to still
// finish exactly on target - see ROTATION_ACCEL_LIMIT's comment above.
RotationProfile computeRotationProfile(
  float rotationDelta_deg,
  float totalTime_sec,
  float accelLimit_degs2) {

  RotationProfile profile;
  profile.totalDelta = rotationDelta_deg;

  float absDelta = fabs(rotationDelta_deg);

  if (totalTime_sec <= 0.0001f || accelLimit_degs2 <= 0.0001f ||
      absDelta <= 0.0001f) {
    // No meaningful ramp possible (or nothing to rotate) - jump straight
    // to the target; angleAtTime() special-cases this the same way.
    profile.accelTime = 0.0f;
    profile.cruiseTime = 0.0f;
    profile.decelTime = 0.0f;
    profile.peakOmegaMag = 0.0f;
    profile.effectiveAccelMag = 0.0f;
    return profile;
  }

  // Max angle a pure triangular (bang-bang) profile could cover in
  // totalTime_sec at accelLimit_degs2: peak velocity V = accelLimit*T/2,
  // distance = V*T/2 = accelLimit*T^2/4.
  float maxTriangleDelta =
    accelLimit_degs2 * totalTime_sec * totalTime_sec / 4.0f;

  if (absDelta <= maxTriangleDelta) {
    // Enough accel headroom - solve the quadratic for peak velocity.
    float a = accelLimit_degs2;
    float T = totalTime_sec;

    float discriminant = (a * T * a * T) - (4.0f * a * absDelta);
    discriminant = max(discriminant, 0.0f); // guard tiny fp negatives

    float peakOmega = ((a * T) - sqrt(discriminant)) / 2.0f;

    float accelTime = peakOmega / a;

    if (accelTime > T / 2.0f) {
      // fp edge case landed us right at the triangle boundary - clamp.
      accelTime = T / 2.0f;
      peakOmega = a * accelTime;
    }

    profile.accelTime = accelTime;
    profile.decelTime = accelTime;
    profile.cruiseTime = T - (2.0f * accelTime);
    profile.peakOmegaMag = peakOmega;
    profile.effectiveAccelMag = a;
  } else {
    // Not enough time/accel to cover absDelta even triangular at the
    // configured limit - widen the effective accel so a pure triangular
    // profile still lands exactly on target by totalTime_sec.
    // Triangle: peak = 2*absDelta/T, accelTime = T/2,
    // effectiveAccel = peak/accelTime = 4*absDelta/T^2.
    profile.accelTime = totalTime_sec / 2.0f;
    profile.decelTime = totalTime_sec / 2.0f;
    profile.cruiseTime = 0.0f;
    profile.peakOmegaMag = (2.0f * absDelta) / totalTime_sec;
    profile.effectiveAccelMag =
      (4.0f * absDelta) / (totalTime_sec * totalTime_sec);
  }

  return profile;
}

// Returns the signed angle (deg) covered from the start of the move up
// to elapsed time t_sec, following the precomputed rotation profile.
// This is the closed-form integral of the trapezoidal angular-velocity
// profile, so desiredHeadingDeg = startYaw + angleAtTime(...) gives a
// smooth, accel-limited heading setpoint rather than snapping.
float angleAtTime(
  const RotationProfile &profile,
  float t_sec,
  float totalTime_sec) {

  float absDelta = fabs(profile.totalDelta);

  if (totalTime_sec <= 0.0001f || absDelta <= 0.0001f) {
    return profile.totalDelta;
  }

  float sign = (profile.totalDelta >= 0.0f) ? 1.0f : -1.0f;

  t_sec = constrain(t_sec, 0.0f, totalTime_sec);

  float mag;
  float a = profile.effectiveAccelMag;

  if (t_sec < profile.accelTime) {
    mag = 0.5f * a * t_sec * t_sec;
  } else {
    float cruiseEnd = profile.accelTime + profile.cruiseTime;
    float magAtAccelEnd = 0.5f * a * profile.accelTime * profile.accelTime;

    if (t_sec < cruiseEnd) {
      mag = magAtAccelEnd + (profile.peakOmegaMag * (t_sec - profile.accelTime));
    } else {
      float magAtCruiseEnd =
        magAtAccelEnd + (profile.peakOmegaMag * profile.cruiseTime);
      float tIntoDecel = t_sec - cruiseEnd;
      mag =
        magAtCruiseEnd +
        (profile.peakOmegaMag * tIntoDecel) -
        (0.5f * a * tIntoDecel * tIntoDecel);
    }
  }

  mag = constrain(mag, 0.0f, absDelta);

  return sign * mag;
}

// ============================================================
// Move: vector direction + gradual rotation + accel-limited speed
// ============================================================

// direction_deg     - travel direction in the FIELD frame (see drive())
// duration_ms       - how long this move should take, start to stop
// targetRotation_deg - heading the robot should be facing by the end of
//                      the move (same frame as currentYawDeg)
// maxSpeed          - speed ceiling for this move, 0.0-1.0
//
// Speed ramps up/down within ACCEL_LIMIT so the whole move - including
// acceleration and deceleration - fits exactly into duration_ms.
// The heading SETPOINT (desiredHeadingDeg) is likewise ramped from
// whatever the robot is facing right now to targetRotation_deg using a
// trapezoidal angle profile bounded by ROTATION_ACCEL_LIMIT, over the
// same duration_ms, and the heading-hold PID continuously steers toward
// that moving target so the rotation happens smoothly - with bounded
// angular acceleration at both the start and end - instead of snapping.
//
// Translation direction is held fixed in the field frame the whole time
// (drive() handles re-projecting it into the chassis frame every tick),
// so the robot traces a straight line in direction_deg regardless of how
// much it rotates along the way.
void move(
  float direction_deg,
  unsigned long duration_ms,
  float targetRotation_deg,
  float maxSpeed) {

  if (duration_ms == 0) {
    stopAllMotors();
    return;
  }

  updateIMU();

  float startYaw = currentYawDeg;

  // Shortest signed angular distance to rotate over the course of the
  // move (handles the -180/180 wraparound correctly).
  float rotationDelta = angleError(targetRotation_deg, startYaw);

  float totalTime_sec = duration_ms / 1000.0f;

  MoveProfile profile =
    computeMoveProfile(maxSpeed, totalTime_sec, ACCEL_LIMIT);

  RotationProfile rotationProfile =
    computeRotationProfile(rotationDelta, totalTime_sec, ROTATION_ACCEL_LIMIT);

  resetHeadingPID();

  unsigned long moveStart = millis();

  while (true) {
    unsigned long elapsed_ms = millis() - moveStart;

    if (elapsed_ms >= duration_ms) {
      break;
    }

    updateIMU();

    float t_sec = elapsed_ms / 1000.0f;

    float speed =
      speedAtTime(profile, t_sec, totalTime_sec, ACCEL_LIMIT);

    // Accel-limited heading setpoint - ramps up, optionally cruises, and
    // ramps back down in angular velocity, landing exactly on
    // targetRotation_deg by duration_ms.
    desiredHeadingDeg =
      startYaw + angleAtTime(rotationProfile, t_sec, totalTime_sec);

    float rotation = headingCorrection();

    drive(direction_deg, speed, rotation);

    delay(5);
  }

  // Snap the target heading to its final value and come to a full stop.
  // (Speed is already ~0 here because the profile ramps down to 0 by
  // the end of duration_ms, but this guarantees a clean stop.)
  desiredHeadingDeg = startYaw + rotationDelta;

  stopAllMotors();
}

// ============================================================
// Holonomic Drive (field-oriented)
// ============================================================

// IMPORTANT: direction_deg is a FIELD-relative direction - a fixed
// compass-like angle in the world frame, NOT relative to whichever way
// the chassis currently happens to be pointed.
//
// The chassis frame rotates underneath that field direction as the robot
// turns, so before computing wheel speeds we first re-express the field
// direction vector in the chassis's CURRENT body frame, using the live
// IMU heading (currentYawDeg). This is the standard field-oriented
// holonomic transform.
//
// Without this step, vx/vy would be computed as if direction_deg were
// already a chassis-relative angle. That works fine while the robot
// isn't rotating, but the moment rotation is added on top, "forward"
// drags along with the spin instead of staying fixed in the world - the
// translation vector rotates with the chassis, so a commanded straight
// line turns into a curved/arcing path. Re-projecting into the body
// frame every tick is what keeps translation and rotation independent,
// exactly like field-oriented mecanum/omni drives (e.g. RoboCup soccer
// robots, FRC swerve robots in field-centric mode).
void drive(
  float direction_deg,
  float speed,
  float rotation) {
  speed = constrain(speed, 0.0f, 1.0f);

  // Apply the IMU's actual yaw convention (see YAW_SIGN above) before
  // rotating the field-relative direction into the chassis's current
  // body frame.
  float effectiveYaw = YAW_SIGN * currentYawDeg;

  float body_direction_rad =
    (direction_deg - effectiveYaw) * PI / 180.0f;

  float vx =
    speed * cos(body_direction_rad);

  float vy =
    speed * sin(body_direction_rad);

  float wheel_speeds[4];

  // Correct tangential-omniwheel kinematics for a wheel mounted at
  // position angle theta (measured from the chassis forward axis):
  //   v = -vx*sin(theta) + vy*cos(theta) + rotation
  // The previous version had the vx term's sign flipped, which mirrors
  // the realized travel direction left/right relative to what was
  // commanded (it does NOT by itself cause arcing, but it is wrong and
  // worth fixing now that the wheel layout is confirmed).
  wheel_speeds[0] =
    -vx * sin(45 * PI / 180.0f) + vy * cos(45 * PI / 180.0f) + rotation;

  wheel_speeds[1] =
    -vx * sin(-45 * PI / 180.0f) + vy * cos(-45 * PI / 180.0f) + rotation;

  wheel_speeds[2] =
    -vx * sin(-135 * PI / 180.0f) + vy * cos(-135 * PI / 180.0f) + rotation;

  wheel_speeds[3] =
    -vx * sin(135 * PI / 180.0f) + vy * cos(135 * PI / 180.0f) + rotation;

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

#ifdef DEBUG_MOVE
  static unsigned long lastDebugMs = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastDebugMs >= 100) {
    lastDebugMs = nowMs;
    Serial.print("dir=");
    Serial.print(direction_deg);
    Serial.print(" yaw=");
    Serial.print(currentYawDeg);
    Serial.print(" bodyDir=");
    Serial.print(body_direction_rad * 180.0f / PI);
    Serial.print(" vx=");
    Serial.print(vx);
    Serial.print(" vy=");
    Serial.print(vy);
    Serial.print(" rot=");
    Serial.print(rotation);
    Serial.print(" w=[");
    Serial.print(wheel_speeds[0]);
    Serial.print(",");
    Serial.print(wheel_speeds[1]);
    Serial.print(",");
    Serial.print(wheel_speeds[2]);
    Serial.print(",");
    Serial.print(wheel_speeds[3]);
    Serial.println("]");
  }
#endif

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
