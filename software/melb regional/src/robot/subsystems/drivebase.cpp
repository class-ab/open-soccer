#include "drivebase.h"

#include "imu.h"
#include "robot_state.h"
#include "robot_config.h"

// ============================================================
// Motor Pins
// ============================================================

static const int M1a = 2; // FRONT LEFT
static const int M1b = 3;
static const int M2a = 5; // FRONT RIGHT
static const int M2b = 4;
static const int M3a = 9; // BACK LEFT
static const int M3b = 8;
static const int M4a = 7; // BACK RIGHT
static const int M4b = 6;

// Forward declaration - defined further down under "Motor Output", but
// drive() (defined earlier in this file) needs to call it.
static void SetSpeed(int motor, int pwm);

// ============================================================
// Motor Calibration
// ============================================================

// 1.00 = nominal
// >1.00 = increase motor speed
// <1.00 = decrease motor speed

static const float motorMult[5] = {
  1.0f,   // unused
  1.00f,  // M1 FL
  1.00f,  // M2 FR
  1.00f,  // M3 BL
  1.00f   // M4 BR
};

// ============================================================
// Move / Acceleration Settings
// ============================================================

// Hard ceiling on how fast speed is allowed to ramp, in speed-fraction
// per second (speed is a 0.0-1.0 fraction of full motor output).
//
// Example: ACCEL_LIMIT = 1.5 means going from 0 -> 1.0 speed takes
// 1/1.5 = 0.667 seconds, at minimum.
//
// Every move() ramps speed up to its own maxSpeed at this rate, holds
// it (if there's time left), then ramps back down to 0 - all timed so
// the whole profile exactly fits the requested duration. If duration is
// too short to reach maxSpeed at this accel limit, the move
// automatically falls back to a triangular profile (speed peaks below
// maxSpeed) rather than violating the acceleration limit.
static const float ACCEL_LIMIT = 1.1f;

// Hard ceilings on the heading SETPOINT (desiredHeadingDeg): how fast its
// angular velocity is allowed to ramp (ROTATION_ACCEL_LIMIT, deg/s^2) and
// how fast it's allowed to spin once ramped up (ROTATION_MAX_SPEED, deg/s).
// Analogous to ACCEL_LIMIT above, but for rotation.
//
// Rotation and translation are deliberately decoupled here: translation's
// speed profile is solved to fit duration_ms exactly (see ACCEL_LIMIT
// above), but the rotation profile is solved for its OWN minimum
// completion time - ramp up to ROTATION_MAX_SPEED (or as close as the
// angle allows), cruise, ramp down - independent of duration_ms entirely.
// If that finishes before the move ends, desiredHeadingDeg just holds at
// the target for the remainder (zero heading error, PID output ~0) while
// translation keeps going. This means a big rotation paired with a short
// move happens as soon and as fast as the robot is physically willing to
// spin, rather than being artificially stretched out to fill the whole
// move - and a small rotation on a long move finishes quickly and holds,
// rather than crawling around at a barely-perceptible cruise rate for the
// entire move (which is what the old duration-filling profile did).
//
// This is effectively "two independent processes" (translation timed by
// duration_ms, rotation timed by its own physics) running in the same
// control loop rather than two separate real threads/tasks - much
// simpler, and sufficient here since both are just read out by elapsed
// time on every 5ms tick regardless.
//
// One consequence: if duration_ms is too short for the rotation to
// physically finish at these limits, the move will simply end with the
// heading wherever the profile got to - it is NOT forced to snap to the
// target at the last moment. The next move() call reads the true current
// yaw as its new starting point, so this never desyncs anything; it just
// means very large rotations need either a longer move or a higher
// ROTATION_MAX_SPEED/ROTATION_ACCEL_LIMIT to fully complete within one
// move.
static int COUNT = 0; // currently unused, carried over from the original sketch
static const float ROTATION_ACCEL_LIMIT = 720.0f; // deg/s^2
static const float ROTATION_MAX_SPEED = 240.0f;   // deg/s

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

static const float HEADING_KP = 0.005f;
static const float HEADING_KI = 0.0f;
static const float HEADING_KD = 0.001f;

// Clamp on the accumulated integral term (anti-windup), expressed in the
// same units as the correction output (-1.0 .. 1.0 motor scale).
static const float HEADING_INTEGRAL_MAX = 0.20f;

// Set to +1.0 or -1.0 depending on which way your IMU's yaw increases.
// TEST: hold the robot still and rotate it BY HAND counter-clockwise as
// seen from directly above (bird's-eye view). Watch currentYawDeg over
// Serial. If it INCREASES, leave this at +1.0. If it DECREASES, change
// this to -1.0. Getting this backwards is the single most common cause
// of a holonomic robot arcing instead of going straight while rotating.
const float YAW_SIGN = 1.0f;

float desiredHeadingDeg = 0.0f;

// PID internal state
static float headingIntegral = 0.0f;
static float headingLastError = 0.0f;
static unsigned long headingLastTimeMs = 0;
static bool headingPidInitialized = false;

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

// Unlike MoveProfile (which ramps a free-running speed for a fixed
// duration with no distance requirement), a RotationProfile solves for
// the MINIMUM time needed to cover rotationDelta at ROTATION_ACCEL_LIMIT
// / ROTATION_MAX_SPEED - it is NOT solved against the move's duration_ms
// at all. rotationTime is however long that actually takes; the caller
// (move()) just holds at the target once elapsed time passes rotationTime.
struct RotationProfile {
  float accelTime;         // seconds spent ramping angular velocity up
  float cruiseTime;        // seconds spent holding peakOmegaMag (0 if triangular)
  float decelTime;         // seconds spent ramping angular velocity down
  float peakOmegaMag;      // magnitude of peak angular velocity reached, deg/s
  float effectiveAccelMag; // magnitude of accel used, deg/s^2 (== ROTATION_ACCEL_LIMIT)
  float totalDelta;        // signed total rotation this profile covers, deg
  float rotationTime;      // accelTime + cruiseTime + decelTime, seconds -
                            // this profile's own completion time, independent
                            // of the enclosing move's duration_ms
};

// ============================================================
// Heading Helpers
// ============================================================

static float angleError(
  float target,
  float current) {
  float error = target - current;

  while (error > 180.0f)
    error -= 360.0f;

  while (error < -180.0f)
    error += 360.0f;

  return error;
}

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
static MoveProfile computeMoveProfile(
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
static float speedAtTime(
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

// Solves for the MINIMUM-TIME trapezoidal (or triangular) angular
// profile that covers rotationDelta_deg, ramping at accelLimit_degs2 up
// to at most maxOmega_degs before ramping back down - the rotation
// equivalent of "get there as fast as these limits allow," not "spread
// evenly across however long the move happens to take."
//
// Distance covered by ramping straight up to maxOmega and immediately
// back down with no cruise (a pure triangle) is maxOmega^2 / accelLimit.
// If the requested angle fits under that, the peak is never reached -
// solve the symmetric triangle instead (t_a = sqrt(absDelta / accelLimit)).
// Otherwise it's a full trapezoid: ramp to maxOmega, cruise however long
// is needed to cover the remaining distance, then ramp back down.
//
// rotationTime (accelTime + cruiseTime + decelTime) is this profile's own
// completion time - it has nothing to do with the enclosing move's
// duration_ms. angleAtTime() clamps to it, so once a move's elapsed time
// passes rotationTime, the heading setpoint just holds at the target.
static RotationProfile computeRotationProfile(
  float rotationDelta_deg,
  float accelLimit_degs2,
  float maxOmega_degs) {

  RotationProfile profile;
  profile.totalDelta = rotationDelta_deg;

  float absDelta = fabs(rotationDelta_deg);
  float maxOmega = fabs(maxOmega_degs);

  if (absDelta <= 0.0001f || accelLimit_degs2 <= 0.0001f ||
      maxOmega <= 0.0001f) {
    // Nothing to rotate, or no meaningful ramp possible - treat as
    // already-arrived; angleAtTime() special-cases this the same way.
    profile.accelTime = 0.0f;
    profile.cruiseTime = 0.0f;
    profile.decelTime = 0.0f;
    profile.peakOmegaMag = 0.0f;
    profile.effectiveAccelMag = 0.0f;
    profile.rotationTime = 0.0f;
    return profile;
  }

  float a = accelLimit_degs2;
  float triangleDist = (maxOmega * maxOmega) / a;

  if (absDelta <= triangleDist) {
    // Triangular: accelerates the whole first half, decelerates the
    // whole second half, never actually reaches maxOmega.
    float t_a = sqrt(absDelta / a);

    profile.accelTime = t_a;
    profile.decelTime = t_a;
    profile.cruiseTime = 0.0f;
    profile.peakOmegaMag = a * t_a;
    profile.effectiveAccelMag = a;
    profile.rotationTime = 2.0f * t_a;
  } else {
    // Trapezoid: ramps to maxOmega, cruises at maxOmega long enough to
    // cover the remaining distance, then ramps back down.
    float t_a = maxOmega / a;
    float cruiseDist = absDelta - triangleDist;
    float cruiseTime = cruiseDist / maxOmega;

    profile.accelTime = t_a;
    profile.decelTime = t_a;
    profile.cruiseTime = cruiseTime;
    profile.peakOmegaMag = maxOmega;
    profile.effectiveAccelMag = a;
    profile.rotationTime = (2.0f * t_a) + cruiseTime;
  }

  return profile;
}

// Returns the signed angle (deg) covered from the start of the move up
// to elapsed time t_sec, following the precomputed rotation profile.
// This is the closed-form integral of the trapezoidal angular-velocity
// profile, so desiredHeadingDeg = startYaw + angleAtTime(...) gives a
// smooth, accel-limited heading setpoint rather than snapping.
//
// t_sec is clamped to the PROFILE'S OWN rotationTime, not to the
// enclosing move's duration - once the rotation is done, this simply
// keeps returning the full delta (i.e. the setpoint holds at target)
// for as long as the caller keeps asking, however much longer the move
// itself still has left to run.
static float angleAtTime(
  const RotationProfile &profile,
  float t_sec) {

  float absDelta = fabs(profile.totalDelta);

  if (profile.rotationTime <= 0.0001f || absDelta <= 0.0001f) {
    return profile.totalDelta;
  }

  float sign = (profile.totalDelta >= 0.0f) ? 1.0f : -1.0f;

  t_sec = constrain(t_sec, 0.0f, profile.rotationTime);

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
// Move: vector direction + accel-limited speed + independently
//       accel-limited, as-fast-as-possible rotation
// ============================================================

// direction_deg     - travel direction in the FIELD frame (see drive())
// duration_ms       - how long this move should take, start to stop
// targetRotation_deg - heading the robot should be facing (same frame
//                      as currentYawDeg); reached as soon as physically
//                      possible, not necessarily exactly at move's end
// maxSpeed          - speed ceiling for this move, 0.0-1.0
//
// Speed ramps up/down within ACCEL_LIMIT so the whole move - including
// acceleration and deceleration - fits exactly into duration_ms.
//
// Rotation is handled independently: the heading SETPOINT
// (desiredHeadingDeg) ramps toward targetRotation_deg as fast as
// ROTATION_ACCEL_LIMIT/ROTATION_MAX_SPEED allow, starting the instant
// the move begins, and simply holds at the target once it gets there -
// it does NOT wait around to spread the rotation across the whole
// duration_ms the way translation's speed profile intentionally does.
// A big rotation on a short move happens as fast as the robot can spin;
// a small rotation on a long move finishes quickly and holds steady for
// the rest of the move, rather than crawling the whole way through it.
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
    computeRotationProfile(rotationDelta, ROTATION_ACCEL_LIMIT, ROTATION_MAX_SPEED);

  resetHeadingPID();

  unsigned long moveStart = millis();

  while (true) {
    unsigned long elapsed_ms = millis() - moveStart;

    if (elapsed_ms >= duration_ms) {
      break;
    }

    systemTick(); // battery check can trigger emergencyShutdown() here too

    updateIMU();

    float t_sec = elapsed_ms / 1000.0f;

    float speed =
      speedAtTime(profile, t_sec, totalTime_sec, ACCEL_LIMIT);

    // Accel-limited heading setpoint, timed independently of the move's
    // own duration - ramps toward targetRotation_deg as fast as
    // ROTATION_ACCEL_LIMIT/ROTATION_MAX_SPEED allow, then holds there
    // once rotationProfile.rotationTime elapses (angleAtTime() clamps
    // internally, so this call is valid for the whole move regardless
    // of how it compares to rotationTime).
    desiredHeadingDeg =
      startYaw + angleAtTime(rotationProfile, t_sec);

    float rotation = headingCorrection();

    drive(direction_deg, speed, rotation);

    delay(5);
  }

  // No forced heading snap here (unlike the old duration-filling
  // profile): desiredHeadingDeg is left wherever the rotation profile
  // actually got to. In the common case that's already exactly
  // startYaw + rotationDelta, because the rotation finished and held
  // well before duration_ms ran out. If duration_ms was too short for
  // ROTATION_ACCEL_LIMIT/ROTATION_MAX_SPEED to finish the full rotation,
  // it's left partway there rather than artificially jumped to target -
  // the next move() reads the true current yaw as its new startYaw, so
  // this never desyncs anything.
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
// Motor Output
// ============================================================

static void SetSpeed(int motor, int pwm) {
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

void stopAllMotors() {
  SetSpeed(1, 0);
  SetSpeed(2, 0);
  SetSpeed(3, 0);
  SetSpeed(4, 0);
}

void initDrivebase() {
  pinMode(M1a, OUTPUT);
  pinMode(M1b, OUTPUT);

  pinMode(M2a, OUTPUT);
  pinMode(M2b, OUTPUT);

  pinMode(M3a, OUTPUT);
  pinMode(M3b, OUTPUT);

  pinMode(M4a, OUTPUT);
  pinMode(M4b, OUTPUT);
}
