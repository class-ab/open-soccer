/*
  RCJ Soccer Open - Teensy 4.1 control pipeline
  ------------------------------------------------------------
  Vision@60Hz(OpenMV+hyperbolic mirror) -> BallFilter -> BallVelocity
  -> BehaviorFSM -> TargetPoseGenerator -> PosePID -> OmniKinematics -> Motors

  Vision link: bit-banged, CS-framed serial link from the OpenMV cam.
  OpenMV already unwarps the mirror image and sends ball + goal as
  polar coordinates (radius px, angle deg, 0=robot-forward, +=right)
  relative to the robot's own center.

  NOTE: battery monitoring / OLED status (present on the original demo
  hardware) are intentionally left out here to keep this file focused
  on the control pipeline - bolt them back on separately if needed.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// ============================================================
// Shared types (declared up front - the Arduino IDE auto-generates
// function prototypes and inserts them near the top of the file,
// above where these were originally defined further down, which
// breaks compilation. Keeping them here avoids that.)
// ============================================================
struct VisionData {
  bool ballSeen, goalSeen;
  float ballAng, ballR, goalAng, goalR; // deg, px
};
struct Vec2 { float x, y; };
struct TrackedObject { Vec2 pos, vel; bool valid; unsigned long lastMs; };
struct Pose { float directionDeg, headingDeg, speed; };

// ============================================================
// Motors  (wheel order must match WHEEL_ANGLE_DEG - verify vs your wiring)
// ============================================================
const int MA[4] = {2, 5, 9, 7};
const int MB[4] = {3, 4, 8, 6};
const float WHEEL_ANGLE_DEG[4] = {45, -45, -135, 135};

void motorWrite(int i, int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0) { analogWrite(MA[i], pwm);  analogWrite(MB[i], 0); }
  else          { analogWrite(MA[i], 0);    analogWrite(MB[i], -pwm); }
}
void motorsStop() { for (int i = 0; i < 4; i++) motorWrite(i, 0); }

// ============================================================
// IMU (yaw only - needed to stabilize the robot-relative sensor data
// and for heading hold)
// ============================================================
Adafruit_BNO08x bno(-1);
sh2_SensorValue_t sv;
float yawDeg = 0;

void imuBegin() {
  bno.begin_I2C();
  bno.enableReport(SH2_GAME_ROTATION_VECTOR);
}
void imuUpdate() {
  if (bno.wasReset()) bno.enableReport(SH2_GAME_ROTATION_VECTOR);
  if (bno.getSensorEvent(&sv) && sv.sensorId == SH2_GAME_ROTATION_VECTOR) {
    float r = sv.un.gameRotationVector.real, i = sv.un.gameRotationVector.i,
          j = sv.un.gameRotationVector.j,   k = sv.un.gameRotationVector.k;
    yawDeg = atan2(2 * (r * k + i * j), 1 - 2 * (j * j + k * k)) * 180.0f / PI;
  }
}
float wrap180(float a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }

// ============================================================
// Vision link - bit-banged, CS-framed (SCK=13, MOSI=11, CS=10, MISO=12)
// Packet (12 bytes): sync, flags, ballAng*100(i16), ballR(u16),
//                     goalAng*100(i16), goalR(u16), pad, checksum(xor)
// ============================================================
const int V_SCK = 13, V_MOSI = 11, V_CS = 10, V_MISO = 12;
const uint8_t V_SYNC = 0xC5, V_LEN = 12;

volatile uint8_t vBitCnt = 0, vByteIdx = 0, vByte = 0;
volatile uint8_t vBuf[V_LEN], vReadyBuf[V_LEN];
volatile bool vPacketReady = false;

void visionClockISR() {
  vByte <<= 1;
  if (digitalReadFast(V_MOSI)) vByte |= 1;
  if (++vBitCnt == 8) {
    if (vByteIdx < V_LEN) vBuf[vByteIdx++] = vByte;
    vBitCnt = 0; vByte = 0;
  }
}
void visionCSISR() {
  if (digitalReadFast(V_CS) == LOW) {
    vBitCnt = 0; vByteIdx = 0; vByte = 0;
  } else if (vByteIdx == V_LEN) {
    for (int i = 0; i < V_LEN; i++) vReadyBuf[i] = vBuf[i];
    vPacketReady = true;
  }
}

volatile VisionData latestVision = {false, false, 0, 0, 0, 0};
volatile unsigned long lastVisionMs = 0;

void visionDecode() {
  if (!vPacketReady) return;
  vPacketReady = false;
  uint8_t b[V_LEN];
  noInterrupts(); for (int i = 0; i < V_LEN; i++) b[i] = vReadyBuf[i]; interrupts();

  uint8_t cs = 0;
  for (int i = 0; i < V_LEN - 1; i++) cs ^= b[i];
  if (cs != b[V_LEN - 1] || b[0] != V_SYNC) return; // corrupt/unsynced, drop

  latestVision.ballSeen = b[1] & 0x01;
  latestVision.goalSeen = b[1] & 0x02;
  latestVision.ballAng = ((int16_t)((b[2] << 8) | b[3])) / 100.0f;
  latestVision.ballR   = (float)((uint16_t)((b[4] << 8) | b[5]));
  latestVision.goalAng = ((int16_t)((b[6] << 8) | b[7])) / 100.0f;
  latestVision.goalR   = (float)((uint16_t)((b[8] << 8) | b[9]));
  lastVisionMs = millis();
}
void getVision(VisionData &out) {
  noInterrupts();
  out.ballSeen = latestVision.ballSeen;
  out.goalSeen = latestVision.goalSeen;
  out.ballAng  = latestVision.ballAng;
  out.ballR    = latestVision.ballR;
  out.goalAng  = latestVision.goalAng;
  out.goalR    = latestVision.goalR;
  interrupts();
}

// ============================================================
// Ball / goal filter + velocity estimate
// Robot-relative polar readings are rotated into a yaw-locked frame
// before filtering, so the filter isn't corrupted by the robot's own
// spin (a stationary ball must filter to a stationary point even while
// the robot rotates around it).
// ============================================================
Vec2 polarToLockedFrame(float r, float angChassis, float yaw) {
  float a = (angChassis + yaw) * PI / 180.0f;
  return { r * cos(a), r * sin(a) };
}
void lockedFrameToChassisPolar(Vec2 p, float yaw, float &ang, float &r) {
  r = sqrt(p.x * p.x + p.y * p.y);
  ang = wrap180(atan2(p.y, p.x) * 180.0f / PI - yaw);
}

TrackedObject ballTrk = {{0,0},{0,0}, false, 0};
TrackedObject goalTrk = {{0,0},{0,0}, false, 0};

const float POS_FILTER_ALPHA = 0.5f;
const float VEL_FILTER_ALPHA = 0.3f;
const unsigned long OBJ_TIMEOUT_MS = 300;

void updateTracked(TrackedObject &t, bool seen, float r, float ang) {
  unsigned long now = millis();
  if (!seen) {
    if (now - t.lastMs > OBJ_TIMEOUT_MS) t.valid = false;
    return;
  }
  Vec2 raw = polarToLockedFrame(r, ang, yawDeg);
  if (!t.valid) {
    t.pos = raw; t.vel = {0, 0};
  } else {
    float dt = (now - t.lastMs) / 1000.0f;
    Vec2 filtered = { t.pos.x + POS_FILTER_ALPHA * (raw.x - t.pos.x),
                      t.pos.y + POS_FILTER_ALPHA * (raw.y - t.pos.y) };
    if (dt > 0.001f) {
      Vec2 v = { (filtered.x - t.pos.x) / dt, (filtered.y - t.pos.y) / dt };
      t.vel.x += VEL_FILTER_ALPHA * (v.x - t.vel.x);
      t.vel.y += VEL_FILTER_ALPHA * (v.y - t.vel.y);
    }
    t.pos = filtered;
  }
  t.valid = true; t.lastMs = now;
}

// ============================================================
// Behavior state machine
// ============================================================
enum Behavior { SEARCH, CHASE, ORBIT, ATTACK };
Behavior behavior = SEARCH;

const float CLOSE_R_PX = 120;       // "ball is close enough to line up" threshold
const float ALIGN_THRESH_DEG = 15;  // robot/ball/goal angular alignment tolerance

// Strategy tree: extend with more branches/states here as needed.
void updateBehavior() {
  if (!ballTrk.valid) { behavior = SEARCH; return; }

  float bAng, bR;
  lockedFrameToChassisPolar(ballTrk.pos, yawDeg, bAng, bR);

  if (bR > CLOSE_R_PX) { behavior = CHASE; return; }

  if (!goalTrk.valid) { behavior = ATTACK; return; } // no goal ref: just push forward

  float gAng, gR;
  lockedFrameToChassisPolar(goalTrk.pos, yawDeg, gAng, gR);
  float alignErr = wrap180(gAng - bAng);

  behavior = (fabs(alignErr) > ALIGN_THRESH_DEG) ? ORBIT : ATTACK;
}

// ============================================================
// Target pose generator
// Pose = desired drive direction (field/yaw-locked deg), desired
// heading (deg), desired speed (0-1). No absolute (x,y) localization
// is used - everything is expressed relative to current yaw, matching
// what the sensor suite actually provides.
// ============================================================
const float SEARCH_SPIN_DEG_PER_TICK = 0.3f; // ~60 deg/s at a 5ms tick
const float CHASE_SPEED  = 0.55f;
const float ORBIT_SPEED  = 0.30f;
const float ATTACK_SPEED = 0.65f;

float searchHeadingOffset = 0;

Pose poseSearch() {
  searchHeadingOffset = wrap180(searchHeadingOffset + SEARCH_SPIN_DEG_PER_TICK);
  return { yawDeg, yawDeg + searchHeadingOffset, 0.0f };
}
Pose poseChase(float bAng) {
  return { yawDeg + bAng, yawDeg + bAng, CHASE_SPEED };
}
Pose poseOrbit(float bAng, float alignErr) {
  float sign = (alignErr >= 0) ? 1.0f : -1.0f;
  return { yawDeg + bAng + sign * 90.0f, yawDeg + bAng, ORBIT_SPEED };
}
Pose poseAttack(float bAng) {
  return { yawDeg + bAng, yawDeg + bAng, ATTACK_SPEED };
}

// Strategy tree: add branches/pose functions here for new behaviors.
Pose generateTargetPose() {
  if (behavior == SEARCH) { searchHeadingOffset = wrap180(searchHeadingOffset); return poseSearch(); }

  float bAng, bR;
  lockedFrameToChassisPolar(ballTrk.pos, yawDeg, bAng, bR);

  if (behavior == CHASE) return poseChase(bAng);

  if (behavior == ORBIT) {
    float gAng, gR;
    lockedFrameToChassisPolar(goalTrk.pos, yawDeg, gAng, gR);
    return poseOrbit(bAng, wrap180(gAng - bAng));
  }

  return poseAttack(bAng); // ATTACK
}

// ============================================================
// Pose controller (heading PID; translation speed comes straight
// from the pose generator's ramp/behavior logic above)
// ============================================================
float headingIntegral = 0, headingLastErr = 0;
unsigned long headingLastMs = 0;
const float KP = 0.006f, KI = 0.0f, KD = 0.0015f;

float headingPID(float targetHeadingDeg) {
  unsigned long now = millis();
  float err = wrap180(targetHeadingDeg - yawDeg);
  float dt = (headingLastMs == 0) ? 0 : (now - headingLastMs) / 1000.0f;

  if (dt > 0) headingIntegral = constrain(headingIntegral + err * dt, -0.2f, 0.2f);
  float deriv = (dt > 0) ? (err - headingLastErr) / dt : 0;

  headingLastErr = err; headingLastMs = now;
  return constrain(err * KP + headingIntegral * KI + deriv * KD, -0.4f, 0.4f);
}

// ============================================================
// Omni-wheel kinematics + motor output
// ============================================================
void driveChassis(float directionDeg, float speed, float rotation) {
  speed = constrain(speed, 0.0f, 1.0f);
  float bodyDir = (directionDeg - yawDeg) * PI / 180.0f;
  float vx = speed * cos(bodyDir), vy = speed * sin(bodyDir);

  float w[4], maxW = 0;
  for (int i = 0; i < 4; i++) {
    float th = WHEEL_ANGLE_DEG[i] * PI / 180.0f;
    w[i] = -vx * sin(th) + vy * cos(th) + rotation;
    maxW = max(maxW, fabs(w[i]));
  }
  if (maxW > 1.0f) for (int i = 0; i < 4; i++) w[i] /= maxW;
  for (int i = 0; i < 4; i++) motorWrite(i, w[i] * 255);
}

// ============================================================
// Setup / Loop
// ============================================================
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) { pinMode(MA[i], OUTPUT); pinMode(MB[i], OUTPUT); }

  pinMode(V_MOSI, INPUT); pinMode(V_SCK, INPUT); pinMode(V_CS, INPUT); pinMode(V_MISO, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(V_CS), visionCSISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(V_SCK), visionClockISR, RISING);

  Wire.begin();
  imuBegin();
  delay(300);
  imuUpdate();
}

void loop() {
  visionDecode();
  imuUpdate();

  VisionData v; getVision(v);
  updateTracked(ballTrk, v.ballSeen, v.ballR, v.ballAng);
  updateTracked(goalTrk, v.goalSeen, v.goalR, v.goalAng);

  updateBehavior();
  Pose target = generateTargetPose();
  float rotation = headingPID(target.headingDeg);
  driveChassis(target.directionDeg, target.speed, rotation);

  delay(5); // ~200Hz control tick
}
