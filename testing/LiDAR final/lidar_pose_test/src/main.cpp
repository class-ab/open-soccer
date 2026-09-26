// ============================================================================
// LD14P + BNO08x lidar/pose test stream  (Teensy 4.1)
//
// Bench-test tool: parses the LD14P scan, tracks heading from the BNO08x,
// and recovers the robot's (x, y) position by matching lidar hits against
// the known field walls (rectangle with two notches) -- no wheel odometry
// needed. Streams every hit plus the fitted pose over USB Serial so a PC
// script can draw a live map. Adapted from the full localization subsystem;
// trimmed to just lidar + IMU + pose (opponent/ball/comms/PID removed).
//
// Wiring (Teensy 4.1):
//   LD14P lidar -> Serial3  (pin 15 = RX3, pin 14 = TX3), 5V, GND
//   BNO08x IMU  -> I2C      (pin 18 = SDA, pin 19 = SCL), 3.3V, GND
//
// Library needed (Arduino Library Manager): "Adafruit BNO08x"
// (pulls in Adafruit_Sensor / Adafruit_BusIO as dependencies).
//
// PlatformIO source: src/main.cpp; shared type definitions are in include/.
//
// PC side: run lidar_viewer.py (pip install pyserial matplotlib) to view live.
//
// Serial protocol -- two record kinds, mixed on the wire:
//   "P <x_mm> <y_mm> <heading_deg> <quality 0-1>\n"   ASCII pose update (~50 Hz)
//   Lines starting with '#' are status/comments -- safe to ignore.
//   Lidar hits are sent as a binary frame (not ASCII, no line terminator):
//     byte 0        : 0xAA sync byte (never a valid ASCII line-start)
//     byte 1        : point count N (0-12)
//     N * 4 bytes   : (int16 x_mm, int16 y_mm) little-endian, world frame
//     byte last     : XOR checksum of count byte + all payload bytes
//   One frame is sent per LD14P packet instead of one Serial write per point,
//   to keep per-point formatting/call overhead off the localization hot path.
//
// Tuning knobs you may want to touch for your setup:
//   YAW_SIGN         - flip to -1 if on-screen rotation is mirrored
//   BNO08X_RESET      - set to your reset pin if you wired one (else -1)
//   POLE_ANGLES_DEG   - angular sectors blocked by robot structure; clear/adjust
//                       if your bench rig has no such obstruction
// ============================================================================

#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "lidar_pose_test_types.h"

// ---------------------------------------------------------------------------
// Hardware configuration
// ---------------------------------------------------------------------------
#define LIDAR_UART Serial3
constexpr uint32_t LIDAR_UART_BAUD = 230400;  // LD14P fixed baud rate
constexpr uint8_t LIDAR_SPEED_CONTROL_PIN = 14;  // Teensy TX3 / LD14P RX
constexpr uint32_t LIDAR_PWM_FREQUENCY_HZ = 1000;
constexpr float LIDAR_PWM_ENTRY_DUTY_PERCENT = 50.0f;
constexpr float LIDAR_PWM_MIN_DUTY_PERCENT = 45.1f;
constexpr float LIDAR_PWM_MAX_DUTY_PERCENT = 80.0f;
constexpr float LIDAR_TARGET_SPEED_DEG_S = 2880.0f;
constexpr float LIDAR_SPEED_TOLERANCE_DEG_S = 36.0f;
constexpr unsigned long LIDAR_CONTROL_INTERVAL_MS = 500;
constexpr int8_t BNO08X_RESET = -1;           // -1 = no reset pin wired

constexpr float YAW_SIGN = 1.0f;

// ---------------------------------------------------------------------------
// Field geometry (mm) -- must match FIELD_* in lidar_viewer.py
// ---------------------------------------------------------------------------
constexpr float FIELD_WIDTH_MM = 2430.0f;
constexpr float FIELD_HEIGHT_MM = 1820.0f;
constexpr float NOTCH_DEPTH_MM = 226.0f;
constexpr float NOTCH_HEIGHT_MM = 470.0f;

// ---------------------------------------------------------------------------
// LD14P packet format
// ---------------------------------------------------------------------------
constexpr uint8_t PACKET_SIZE = 47;
constexpr uint8_t PACKET_HEADER = 0x54;
constexpr uint8_t PACKET_VERLEN = 0x2C;
constexpr uint8_t POINTS_PER_PACKET = 12;
constexpr size_t LIDAR_RX_BUFFER_SIZE =  16384;

constexpr float MIN_RANGE_MM = 50.0f;
constexpr float MAX_RANGE_MM = 12000.0f;
// DEG_TO_RAD / RAD_TO_DEG are already provided (as double-precision macros)
// by the Teensy core's wiring.h -- used as-is below, not redefined here.
static_assert(POINTS_PER_PACKET == 12,
              "LidarPacket in lidar_pose_test_types.h hardcodes 12 points; update both together.");

// Angular sectors (relative to the lidar's own zero) blocked by physical
// robot structure. Adjust to your chassis, or ignore if unobstructed.
constexpr float POLE_ANGLES_DEG[4] = {45.0f, 135.0f, -135.0f, -45.0f};
constexpr float POLE_HALF_WIDTH_DEG = 4.1f;

// ---------------------------------------------------------------------------
// Scan-matching / ray-window tuning
// ---------------------------------------------------------------------------
constexpr float HUBER_MM = 120.0f;
constexpr float MAX_ACCEPTED_COST = 24000.0f;
constexpr float MAX_FIX_CORRECTION_MM = 35.0f;
constexpr float FIT_CONDITION_MIN = 0.004f;
constexpr uint8_t MIN_FIT_RAYS = 18;
constexpr uint8_t MAX_FIT_RAYS = 72;
// NEW_RAY_TRIGGER/FIT_INTERVAL_MS: raise either to fit less often. Fits this
// close together mostly re-solve nearly the same RAY_WINDOW_MS window, so
// pushing them faster than this adds fit-to-fit noise (jitter) for little
// extra freshness -- RAY_WINDOW_MS, not fit cadence, sets the latency floor.
constexpr uint8_t NEW_RAY_TRIGGER = 6;   // higher = fewer, more-independent fits (less jitter, same latency)
constexpr uint16_t MAX_WINDOW_RAYS = 512;
// RAY_WINDOW_MS: lower = less latency but noisier/fewer-ray fits (raise MIN_FIT_RAYS floor risk);
// higher = smoother fits but more lag (the fit result reflects the window's time-average).
constexpr unsigned long RAY_WINDOW_MS = 35;
constexpr unsigned long FIT_INTERVAL_MS = 10;  // higher = fewer, more-independent fits (less jitter, same latency)
constexpr unsigned long POSE_STREAM_INTERVAL_MS = 20;

// Position tracked with an alpha-beta (g-h) filter instead of a plain
// low-pass, so it carries a velocity estimate and can be predicted forward
// between fits (same idea as the yaw-rate prediction below).
constexpr float POSE_FIX_GAIN = 0.45f;  // g: position correction toward each new fit.
                                         // Higher = snappier but jitterier; lower = smoother but laggier.
constexpr float POSE_VEL_GAIN = 0.12f;  // h: velocity correction toward each new fit.
                                         // Higher = predicts fast motion sooner but amplifies fit noise
                                         // into visible jitter; lower = smoother velocity, slower to react.
// Floor on the dt used only in the velocity update's division (see updatePoseFromLidar()).
// Fits can land only a few ms apart, and dividing a noisy residual by a tiny dt blows the
// noise up into large velocity swings (the dominant jitter source) -- this caps that gain.
// Lower = more responsive velocity to genuinely fast motion, but reintroduces jitter;
// higher = smoother velocity, slower to pick up sudden accelerations.
constexpr float MIN_POSE_VEL_DT_S = 0.02f;
constexpr unsigned long POSE_VEL_TIMEOUT_MS = 150;  // no accepted fit for this long -> assume stopped
constexpr unsigned long MAX_POSE_PREDICTION_MS = 50;

// ---------------------------------------------------------------------------
// IMU: BNO08x game-rotation-vector -> unwrapped yaw + rate
// ---------------------------------------------------------------------------
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

constexpr uint32_t IMU_REPORT_INTERVAL_US = 10000;  // 100 Hz
constexpr float YAW_RATE_FILTER = 0.35f;
constexpr float MAX_YAW_RATE_DEG_S = 900.0f;
constexpr unsigned long MAX_YAW_PREDICTION_MS = 20;
constexpr unsigned long IMU_TIMEOUT_MS = 100;

float currentYawDeg = 0.0f;
float previousRawYawDeg = 0.0f;
float unwrappedYawDeg = 0.0f;
float yawRateDegPerSec = 0.0f;
uint32_t lastYawSampleUs = 0;
bool yawSampleCaptured = false;
bool rebaseNextYawSample = false;

float wrapDegrees(float angle) {
  angle = fmodf(angle + 180.0f, 360.0f);
  if (angle < 0.0f) angle += 360.0f;
  return angle - 180.0f;
}

float quaternionToYawDegrees(float real, float i, float j, float k) {
  const float yaw = atan2f(2.0f * (real * k + i * j), 1.0f - 2.0f * (j * j + k * k));
  return yaw * RAD_TO_DEG;
}

void updateYawFromQuaternion(float rawYawDeg, uint32_t nowUs) {
  if (!yawSampleCaptured) {
    previousRawYawDeg = rawYawDeg;
    unwrappedYawDeg = 0.0f;
    lastYawSampleUs = nowUs;
    yawRateDegPerSec = 0.0f;
    yawSampleCaptured = true;
  } else if (rebaseNextYawSample) {
    previousRawYawDeg = rawYawDeg;
    lastYawSampleUs = nowUs;
    yawRateDegPerSec = 0.0f;
    rebaseNextYawSample = false;
  } else {
    const float delta = wrapDegrees(rawYawDeg - previousRawYawDeg);
    unwrappedYawDeg += delta;
    const uint32_t elapsedUs = nowUs - lastYawSampleUs;
    if (elapsedUs > 0) {
      const float rate = delta * 1000000.0f / elapsedUs;
      const float boundedRate = fmaxf(-MAX_YAW_RATE_DEG_S,
                                      fminf(MAX_YAW_RATE_DEG_S, rate));
      yawRateDegPerSec += YAW_RATE_FILTER * (boundedRate - yawRateDegPerSec);
      lastYawSampleUs = nowUs;
    }
    previousRawYawDeg = rawYawDeg;
  }

  const uint32_t elapsedUs = nowUs - lastYawSampleUs;
  const uint32_t maxPredictionUs = MAX_YAW_PREDICTION_MS * 1000U;
  const uint32_t predictionUs = elapsedUs < maxPredictionUs ? elapsedUs : maxPredictionUs;
  currentYawDeg = wrapDegrees(unwrappedYawDeg + yawRateDegPerSec * predictionUs / 1000000.0f);
}

void setIMUReports() {
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_REPORT_INTERVAL_US)) {
    Serial.println(F("# WARNING: could not enable BNO08x rotation vector"));
  }
}

bool initIMU() {
  if (!bno08x.begin_I2C()) {
    Serial.println(F("# ERROR: BNO08x not found"));
    return false;
  }
  setIMUReports();
  delay(500);

  const unsigned long start = millis();
  while (!yawSampleCaptured && millis() - start < 1000) {
    if (bno08x.getSensorEvent(&sensorValue) &&
        sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      const float raw = quaternionToYawDegrees(
          sensorValue.un.gameRotationVector.real, sensorValue.un.gameRotationVector.i,
          sensorValue.un.gameRotationVector.j, sensorValue.un.gameRotationVector.k);
      updateYawFromQuaternion(raw, micros());
    }
    delay(1);
  }
  Serial.println(yawSampleCaptured ? F("# IMU yaw zeroed at startup")
                                    : F("# WARNING: no IMU sample captured yet"));
  return true;
}

void updateIMU() {
  if (bno08x.wasReset()) {
    setIMUReports();
    rebaseNextYawSample = true;
  }

  bool receivedRotation = false;
  while (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId != SH2_GAME_ROTATION_VECTOR) continue;
    const float raw = quaternionToYawDegrees(
        sensorValue.un.gameRotationVector.real, sensorValue.un.gameRotationVector.i,
        sensorValue.un.gameRotationVector.j, sensorValue.un.gameRotationVector.k);
    updateYawFromQuaternion(raw, micros());
    receivedRotation = true;
  }

  if (!receivedRotation && yawSampleCaptured) {
    const uint32_t elapsedUs = micros() - lastYawSampleUs;
    const uint32_t maxPredictionUs = MAX_YAW_PREDICTION_MS * 1000U;
    const uint32_t predictionUs = elapsedUs < maxPredictionUs ? elapsedUs : maxPredictionUs;
    currentYawDeg = wrapDegrees(unwrappedYawDeg + yawRateDegPerSec * predictionUs / 1000000.0f);
  }
}

bool isIMUHeadingFresh() {
  return yawSampleCaptured && micros() - lastYawSampleUs <= IMU_TIMEOUT_MS * 1000U;
}

// ---------------------------------------------------------------------------
// Field map: rectangle with two notches, used only to scan-match lidar hits
// against known walls so we can recover the robot's (x, y).
// ---------------------------------------------------------------------------
MapEdge mapEdges[12];

void buildFieldMap() {
  const float mapX[12] = {
      0.0f, 0.0f, NOTCH_DEPTH_MM, NOTCH_DEPTH_MM, 0.0f, 0.0f,
      FIELD_WIDTH_MM, FIELD_WIDTH_MM,
      FIELD_WIDTH_MM - NOTCH_DEPTH_MM, FIELD_WIDTH_MM - NOTCH_DEPTH_MM,
      FIELD_WIDTH_MM, FIELD_WIDTH_MM};
  const float mapY[12] = {
      FIELD_HEIGHT_MM,
      FIELD_HEIGHT_MM / 2.0f + NOTCH_HEIGHT_MM / 2.0f,
      FIELD_HEIGHT_MM / 2.0f + NOTCH_HEIGHT_MM / 2.0f,
      FIELD_HEIGHT_MM / 2.0f - NOTCH_HEIGHT_MM / 2.0f,
      FIELD_HEIGHT_MM / 2.0f - NOTCH_HEIGHT_MM / 2.0f,
      0.0f, 0.0f,
      FIELD_HEIGHT_MM / 2.0f - NOTCH_HEIGHT_MM / 2.0f,
      FIELD_HEIGHT_MM / 2.0f - NOTCH_HEIGHT_MM / 2.0f,
      FIELD_HEIGHT_MM / 2.0f + NOTCH_HEIGHT_MM / 2.0f,
      FIELD_HEIGHT_MM / 2.0f + NOTCH_HEIGHT_MM / 2.0f,
      FIELD_HEIGHT_MM};
  for (uint8_t i = 0; i < 12; ++i) {
    const uint8_t next = (i + 1) % 12;
    mapEdges[i] = {mapX[i], mapY[i], mapX[next] - mapX[i], mapY[next] - mapY[i]};
  }
}

bool insideField(float x, float y) {
  if (x < 0.0f || x > FIELD_WIDTH_MM || y < 0.0f || y > FIELD_HEIGHT_MM) return false;
  const float low = FIELD_HEIGHT_MM * 0.5f - NOTCH_HEIGHT_MM * 0.5f;
  const float high = FIELD_HEIGHT_MM * 0.5f + NOTCH_HEIGHT_MM * 0.5f;
  return !(y > low && y < high && (x < NOTCH_DEPTH_MM || x > FIELD_WIDTH_MM - NOTCH_DEPTH_MM));
}

bool rayToField(float x, float y, float dx, float dy, RayHit &hit) {
  float nearest = MAX_RANGE_MM;
  bool found = false;
  bool nearestVertical = false;
  for (const MapEdge &edge : mapEdges) {
    const float denom = dx * edge.ey - dy * edge.ex;
    if (fabsf(denom) < 1e-6f) continue;
    const float ax = edge.ax - x;
    const float ay = edge.ay - y;
    const float dist = (ax * edge.ey - ay * edge.ex) / denom;
    const float along = (ax * dy - ay * dx) / denom;
    if (dist > 0.0f && along >= 0.0f && along <= 1.0f && dist < nearest) {
      nearest = dist;
      nearestVertical = fabsf(edge.ex) < 1e-5f;
      found = true;
    }
  }
  if (found) hit = {nearest, nearestVertical};
  return found;
}

float robustLoss(float residual) {
  const float a = fabsf(residual);
  return a <= HUBER_MM ? residual * residual : 2.0f * HUBER_MM * a - HUBER_MM * HUBER_MM;
}

// ---------------------------------------------------------------------------
// Ray window + Gauss-Newton pose fit
// ---------------------------------------------------------------------------
Ray rays[MAX_WINDOW_RAYS];
uint16_t rayStart = 0, rayCount = 0, newRayCount = 0;

void appendRay(const Ray &ray) {
  uint16_t index;
  if (rayCount < MAX_WINDOW_RAYS) {
    index = (rayStart + rayCount++) % MAX_WINDOW_RAYS;
  } else {
    index = rayStart;
    rayStart = (rayStart + 1) % MAX_WINDOW_RAYS;
  }
  rays[index] = ray;
  if (newRayCount < MAX_WINDOW_RAYS) ++newRayCount;
}

Ray &rayAt(uint16_t logicalIndex) { return rays[(rayStart + logicalIndex) % MAX_WINDOW_RAYS]; }

void pruneRays(unsigned long now) {
  while (rayCount > 0 && now - rayAt(0).timestampMs > RAY_WINDOW_MS) {
    rayStart = (rayStart + 1) % MAX_WINDOW_RAYS;
    --rayCount;
  }
}

FitResult evaluatePose(float x, float y, const Ray *samples, uint8_t count, float totalWeight,
                       bool buildHessian) {
  FitResult fit = {};
  fit.weightSum = totalWeight;

  if (!insideField(x, y)) {
    fit.cost = fit.weightSum > 0.0f ? robustLoss(600.0f) : 1e30f;
    return fit;
  }

  float weightedCost = 0.0f;
  for (uint8_t i = 0; i < count; ++i) {
    const Ray &ray = samples[i];
    RayHit hit;
    if (!rayToField(x, y, ray.dx, ray.dy, hit)) {
      weightedCost += ray.weight * robustLoss(600.0f);
      continue;
    }
    const float residual = ray.range - hit.range;
    weightedCost += ray.weight * robustLoss(residual);
    ++fit.hitCount;

    if (buildHessian) {
      const float rw = ray.weight * (fabsf(residual) > HUBER_MM ? HUBER_MM / fabsf(residual) : 1.0f);
      const float jx = hit.vertical ? ray.invDx : 0.0f;
      const float jy = hit.vertical ? 0.0f : ray.invDy;
      fit.hxx += rw * jx * jx;
      fit.hxy += rw * jx * jy;
      fit.hyy += rw * jy * jy;
      fit.gx += rw * jx * residual;
      fit.gy += rw * jy * residual;
    }
  }
  fit.cost = fit.weightSum > 0.0f ? weightedCost / fit.weightSum : 1e30f;
  return fit;
}

bool wellConditioned(const FitResult &fit) {
  const float trace = fit.hxx + fit.hyy;
  if (trace <= 1e-6f || fit.hitCount < MIN_FIT_RAYS) return false;
  const float det = fit.hxx * fit.hyy - fit.hxy * fit.hxy;
  return det > FIT_CONDITION_MIN * trace * trace;
}

uint8_t collectFitRays(Ray *samples) {
  if (rayCount < MIN_FIT_RAYS) return 0;
  const uint8_t count = rayCount < MAX_FIT_RAYS ? static_cast<uint8_t>(rayCount) : MAX_FIT_RAYS;
  for (uint8_t i = 0; i < count; ++i) {
    const uint16_t src = static_cast<uint16_t>((static_cast<uint32_t>(i) * rayCount) / count);
    samples[i] = rayAt(src);
  }
  return count;
}

bool refinePose(float &x, float &y, const Ray *samples, uint8_t count, float totalWeight,
                 FitResult &result) {
  for (uint8_t iter = 0; iter < 5; ++iter) {
    const FitResult cur = evaluatePose(x, y, samples, count, totalWeight, true);
    if (!wellConditioned(cur)) return false;

    const float det = cur.hxx * cur.hyy - cur.hxy * cur.hxy;
    float stepX = -(cur.hyy * cur.gx - cur.hxy * cur.gy) / det;
    float stepY = -(-cur.hxy * cur.gx + cur.hxx * cur.gy) / det;

    const float len = hypotf(stepX, stepY);
    if (len > 160.0f) {
      const float s = 160.0f / len;
      stepX *= s;
      stepY *= s;
    }
    if (hypotf(stepX, stepY) < 0.5f) break;

    bool improved = false;
    float scale = 1.0f;
    for (uint8_t t = 0; t < 5; ++t) {
      const float cx = x + stepX * scale, cy = y + stepY * scale;
      if (insideField(cx, cy)) {
        const FitResult cand = evaluatePose(cx, cy, samples, count, totalWeight, false);
        if (cand.cost <= cur.cost) {
          x = cx;
          y = cy;
          improved = true;
          break;
        }
      }
      scale *= 0.5f;
    }
    if (!improved) break;
  }

  result = evaluatePose(x, y, samples, count, totalWeight, true);
  return wellConditioned(result);
}

bool coarseSearch(float cx0, float cy0, float halfX, float halfY, float step,
                   const Ray *samples, uint8_t count, float totalWeight, float &bestX, float &bestY) {
  float bestCost = 1e30f;
  bool found = false;
  for (float x = cx0 - halfX; x <= cx0 + halfX; x += step) {
    for (float y = cy0 - halfY; y <= cy0 + halfY; y += step) {
      if (!insideField(x, y)) continue;
      const FitResult fit = evaluatePose(x, y, samples, count, totalWeight, true);
      if (wellConditioned(fit) && fit.cost < bestCost) {
        bestCost = fit.cost;
        bestX = x;
        bestY = y;
        found = true;
      }
    }
  }
  return found;
}

bool poseInitialized = false;
float poseX = FIELD_WIDTH_MM * 0.5f;
float poseY = FIELD_HEIGHT_MM * 0.5f;
float poseVX = 0.0f;  // mm/s, alpha-beta velocity estimate from consecutive fits
float poseVY = 0.0f;
float poseQuality = 0.0f;
unsigned long lastFitMs = 0;      // gates fit *attempt* cadence (loop())
unsigned long lastPoseFixMs = 0;  // timestamp of last accepted fit, for velocity integration/prediction

void updatePoseFromLidar() {
  Ray samples[MAX_FIT_RAYS];
  const uint8_t count = collectFitRays(samples);
  if (count < MIN_FIT_RAYS) return;
  float totalWeight = 0.0f;
  for (uint8_t i = 0; i < count; ++i) totalWeight += samples[i].weight;

  const unsigned long now = millis();
  const float dtS = poseInitialized
                         ? fminf((now - lastPoseFixMs) / 1000.0f, POSE_VEL_TIMEOUT_MS / 1000.0f)
                         : 0.0f;
  // Dead-reckon the warm-start/search center from the last fix using the velocity estimate.
  const float predX = poseX + poseVX * dtS;
  const float predY = poseY + poseVY * dtS;

  float cx = predX, cy = predY;
  FitResult fit = {};

  if (!poseInitialized) {
    if (!coarseSearch(FIELD_WIDTH_MM * 0.5f, FIELD_HEIGHT_MM * 0.5f, FIELD_WIDTH_MM * 0.5f,
                       FIELD_HEIGHT_MM * 0.5f, 100.0f, samples, count, totalWeight, cx, cy)) return;
    if (!refinePose(cx, cy, samples, count, totalWeight, fit)) return;
  } else if (!refinePose(cx, cy, samples, count, totalWeight, fit) || fit.cost > MAX_ACCEPTED_COST) {
    if (!coarseSearch(predX, predY, 300.0f, 300.0f, 75.0f, samples, count, totalWeight, cx, cy) ||
        !refinePose(cx, cy, samples, count, totalWeight, fit)) return;
  }
  if (fit.cost > MAX_ACCEPTED_COST) return;

  const float quality = 1.0f / (1.0f + fit.cost / 25000.0f);
  if (!poseInitialized) {
    poseX = cx;
    poseY = cy;
    poseVX = 0.0f;
    poseVY = 0.0f;
    poseInitialized = true;
    poseQuality = quality;
    lastPoseFixMs = now;
    return;
  }

  float rx = cx - predX;
  float ry = cy - predY;
  const float rNorm = hypotf(rx, ry);
  if (rNorm > MAX_FIX_CORRECTION_MM) {
    // Disagrees sharply with the dead-reckoned prediction -- clamp like an outlier, don't chase it.
    const float s = MAX_FIX_CORRECTION_MM / rNorm;
    rx *= s;
    ry *= s;
  }

  poseX = predX + POSE_FIX_GAIN * rx;
  poseY = predY + POSE_FIX_GAIN * ry;
  // Floored dt keeps back-to-back fits from turning small residual noise into large velocity spikes.
  const float velDtS = fmaxf(dtS, MIN_POSE_VEL_DT_S);
  poseVX += POSE_VEL_GAIN * rx / velDtS;
  poseVY += POSE_VEL_GAIN * ry / velDtS;
  poseQuality += 0.25f * (quality - poseQuality);
  lastPoseFixMs = now;
}

// ---------------------------------------------------------------------------
// LD14P packet parsing
// ---------------------------------------------------------------------------
uint8_t lidarRxStorage[LIDAR_RX_BUFFER_SIZE];
uint8_t packetBuffer[PACKET_SIZE];
uint8_t packetIndex = 0;
ReceiveState receiveState = WAIT_HEADER;

uint16_t readU16(const uint8_t *bytes) {
  return static_cast<uint16_t>(bytes[0]) | (static_cast<uint16_t>(bytes[1]) << 8);
}

uint8_t lidarCrc8(const uint8_t *bytes, uint8_t length) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < length; ++i) {
    crc ^= bytes[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x4D) : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

bool parsePacket(const uint8_t *bytes, LidarPacket &packet) {
  if (bytes[0] != PACKET_HEADER || bytes[1] != PACKET_VERLEN ||
      lidarCrc8(bytes, PACKET_SIZE - 1) != bytes[PACKET_SIZE - 1]) {
    return false;
  }
  packet.speed = readU16(bytes + 2);
  packet.startAngle = readU16(bytes + 4);
  for (uint8_t i = 0; i < POINTS_PER_PACKET; ++i) {
    const uint8_t *point = bytes + 6 + i * 3;
    packet.distanceMm[i] = readU16(point);
    packet.intensity[i] = point[2];
  }
  packet.endAngle = readU16(bytes + 42);
  return true;
}

float wrappedDifference(float a, float b) { return fmodf(a - b + 540.0f, 360.0f) - 180.0f; }

bool angleBlocked(float localAngleDeg) {
  for (float poleAngle : POLE_ANGLES_DEG) {
    if (fabsf(wrappedDifference(localAngleDeg, poleAngle)) <= POLE_HALF_WIDTH_DEG) return true;
  }
  return false;
}

float pointWeight(uint16_t range, uint8_t intensity) {
  if (range < MIN_RANGE_MM || range > MAX_RANGE_MM || intensity < 8) return 0.0f;
  return intensity / 255.0f;
}

// Sends every valid point from one lidar packet as a single binary frame
// (see protocol doc at top of file) instead of one Serial.print per point.
constexpr uint8_t BINARY_FRAME_SYNC = 0xAA;

void streamPointsBatch(const int16_t *coords, uint8_t pointCount) {
  if (pointCount == 0) return;
  const int frameBytes = 2 + static_cast<int>(pointCount) * 4 + 1;
  if (Serial.availableForWrite() < frameBytes) return;

  uint8_t checksum = pointCount;
  Serial.write(BINARY_FRAME_SYNC);
  Serial.write(pointCount);
  for (uint8_t i = 0; i < pointCount; ++i) {
    const uint16_t ux = static_cast<uint16_t>(coords[i * 2]);
    const uint16_t uy = static_cast<uint16_t>(coords[i * 2 + 1]);
    const uint8_t bytes[4] = {static_cast<uint8_t>(ux & 0xFF), static_cast<uint8_t>(ux >> 8),
                              static_cast<uint8_t>(uy & 0xFF), static_cast<uint8_t>(uy >> 8)};
    Serial.write(bytes, 4);
    for (uint8_t b : bytes) checksum ^= b;
  }
  Serial.write(checksum);
}

uint16_t lastLidarSpeedDegS = 0;
unsigned long lastLidarPacketMs = 0;
bool lidarPacketSeen = false;
uint32_t lidarSpeedSumDegS = 0;
uint16_t lidarSpeedSampleCount = 0;
float lidarPwmDutyPercent = LIDAR_PWM_ENTRY_DUTY_PERCENT;
unsigned long lastLidarControlMs = 0;

void writeLidarPwm(float dutyPercent) {
  const uint16_t pwmMax = (1U << 12) - 1;
  const uint16_t pwmDuty = static_cast<uint16_t>(pwmMax * dutyPercent / 100.0f);
  analogWrite(LIDAR_SPEED_CONTROL_PIN, pwmDuty);
}

void updateLidarSpeedController(unsigned long now) {
  if (now - lastLidarControlMs < LIDAR_CONTROL_INTERVAL_MS) return;
  lastLidarControlMs = now;

  if (lidarSpeedSampleCount == 0) return;
  const float measuredSpeed = static_cast<float>(lidarSpeedSumDegS) / lidarSpeedSampleCount;
  lidarSpeedSumDegS = 0;
  lidarSpeedSampleCount = 0;

  const float error = LIDAR_TARGET_SPEED_DEG_S - measuredSpeed;
  if (fabsf(error) <= LIDAR_SPEED_TOLERANCE_DEG_S) {
    Serial.print(F("# lidar_control target_reached speed_hz "));
    Serial.println(measuredSpeed / 360.0f, 2);
    return;
  }

  const float magnitude = fminf(0.5f, fmaxf(0.05f, fabsf(error) / 1000.0f));
  const float direction = error > 0.0f ? 1.0f : -1.0f;
  const float requestedDuty = lidarPwmDutyPercent + direction * magnitude;
  const float boundedDuty = fminf(LIDAR_PWM_MAX_DUTY_PERCENT,
                                  fmaxf(LIDAR_PWM_MIN_DUTY_PERCENT, requestedDuty));

  if (fabsf(boundedDuty - lidarPwmDutyPercent) < 0.001f) {
    Serial.print(F("# lidar_control duty_limit speed_hz "));
    Serial.print(measuredSpeed / 360.0f, 2);
    Serial.print(F(" duty_percent "));
    Serial.println(lidarPwmDutyPercent, 2);
    return;
  }

  lidarPwmDutyPercent = boundedDuty;
  writeLidarPwm(lidarPwmDutyPercent);
  Serial.print(F("# lidar_control speed_hz "));
  Serial.print(measuredSpeed / 360.0f, 2);
  Serial.print(F(" duty_percent "));
  Serial.println(lidarPwmDutyPercent, 2);
}

void reportLidarStatus(unsigned long now) {
  static unsigned long lastReportMs = 0;
  if (now - lastReportMs < 500) return;
  lastReportMs = now;

  Serial.print(F("# lidar_pwm_duty_percent "));
  Serial.println(lidarPwmDutyPercent, 2);
  if (!lidarPacketSeen) {
    Serial.println(F("# lidar_status waiting_for_packets"));
    return;
  }

  const unsigned long packetAgeMs = now - lastLidarPacketMs;
  Serial.print(F("# lidar_speed_deg_s "));
  Serial.println(lastLidarSpeedDegS);
  Serial.print(F("# lidar_speed_hz "));
  Serial.println(lastLidarSpeedDegS / 360.0f, 2);
  Serial.print(F("# lidar_packet_age_ms "));
  Serial.println(packetAgeMs);
  if (packetAgeMs > 500) Serial.println(F("# lidar_status NO_RECENT_PACKETS"));
}

void addPacket(const LidarPacket &packet) {
  const int32_t angleSpan = (static_cast<int32_t>(packet.endAngle) - packet.startAngle + 36000) % 36000;
  const float headingDeg = YAW_SIGN * currentYawDeg;
  const float lidarSweepMs = packet.speed > 0 ? angleSpan * 10.0f / packet.speed : 0.0f;
  const float serialDelayMs = PACKET_SIZE * 10000.0f / LIDAR_UART_BAUD;
  const unsigned long packetEndMs = millis();
  lastLidarSpeedDegS = packet.speed;
  lastLidarPacketMs = packetEndMs;
  lidarPacketSeen = true;
  if (packet.speed > 0) {
    lidarSpeedSumDegS += packet.speed;
    if (lidarSpeedSampleCount < UINT16_MAX) ++lidarSpeedSampleCount;
  }

  int16_t batchCoords[POINTS_PER_PACKET * 2];
  uint8_t batchCount = 0;

  for (uint8_t i = 0; i < POINTS_PER_PACKET; ++i) {
    const int32_t angleHundredths = (packet.startAngle + angleSpan * i / (POINTS_PER_PACKET - 1)) % 36000;
    const float localAngle = angleHundredths * 0.01f;
    if (angleBlocked(localAngle)) continue;

    const float weight = pointWeight(packet.distanceMm[i], packet.intensity[i]);
    if (weight <= 0.0f) continue;

    // LD14P bearings increase clockwise; field coordinates increase CCW.
    const float fraction = i / static_cast<float>(POINTS_PER_PACKET - 1);
    const float ageMs = serialDelayMs * 0.5f + lidarSweepMs * (1.0f - fraction);
    const float pointHeading = headingDeg - YAW_SIGN * yawRateDegPerSec * (ageMs / 1000.0f);
    const float worldRad = (pointHeading - localAngle) * DEG_TO_RAD;
    const float dx = cosf(worldRad);
    const float dy = sinf(worldRad);
    const float range = static_cast<float>(packet.distanceMm[i]);

    appendRay({dx, dy, range, weight, 1.0f / dx, 1.0f / dy,
               packetEndMs - static_cast<unsigned long>(ageMs)});

    if (batchCount < POINTS_PER_PACKET) {
      batchCoords[batchCount * 2] = static_cast<int16_t>(lroundf(poseX + range * dx));
      batchCoords[batchCount * 2 + 1] = static_cast<int16_t>(lroundf(poseY + range * dy));
      ++batchCount;
    }
  }
  streamPointsBatch(batchCoords, batchCount);
}

void initLidar() {
  LIDAR_UART.addMemoryForRead(lidarRxStorage, sizeof(lidarRxStorage));
  LIDAR_UART.begin(LIDAR_UART_BAUD);
  analogWriteResolution(12);
  analogWriteFrequency(LIDAR_SPEED_CONTROL_PIN, LIDAR_PWM_FREQUENCY_HZ);
    lidarPwmDutyPercent = LIDAR_PWM_ENTRY_DUTY_PERCENT;
    writeLidarPwm(lidarPwmDutyPercent);
  delay(150);
  Serial.print(F("# LD14P PWM frequency_hz "));
  Serial.println(LIDAR_PWM_FREQUENCY_HZ);
    Serial.print(F("# LD14P PWM entry_duty_percent "));
    Serial.println(lidarPwmDutyPercent, 1);
    Serial.println(F("# LD14P speed controller target_hz 8.00 duty_limit_percent 60.00"));
    lastLidarControlMs = millis();
}

void consumeLidar() {
  while (LIDAR_UART.available() > 0) {
    const int incoming = LIDAR_UART.read();
    if (incoming < 0) break;
    const uint8_t value = static_cast<uint8_t>(incoming);

    switch (receiveState) {
      case WAIT_HEADER:
        if (value == PACKET_HEADER) {
          packetBuffer[0] = value;
          packetIndex = 1;
          receiveState = WAIT_LENGTH;
        }
        break;
      case WAIT_LENGTH:
        if (value == PACKET_VERLEN) {
          packetBuffer[1] = value;
          packetIndex = 2;
          receiveState = READ_PACKET;
        } else if (value == PACKET_HEADER) {
          packetBuffer[0] = value;
          packetIndex = 1;
        } else {
          packetIndex = 0;
          receiveState = WAIT_HEADER;
        }
        break;
      case READ_PACKET:
        if (packetIndex >= PACKET_SIZE) {
          packetIndex = 0;
          receiveState = WAIT_HEADER;
          break;
        }
        packetBuffer[packetIndex++] = value;
        if (packetIndex == PACKET_SIZE) {
          LidarPacket packet;
          if (parsePacket(packetBuffer, packet) && isIMUHeadingFresh()) addPacket(packet);
          packetIndex = 0;
          receiveState = WAIT_HEADER;
        }
        break;
    }
  }
}

// ---------------------------------------------------------------------------
unsigned long lastPoseStreamMs = 0;

void streamPose() {
  // Predict forward from the last accepted fit so the streamed pose reflects "now".
  const unsigned long heldMs = poseInitialized ? millis() - lastPoseFixMs : 0;
  const unsigned long predMs = heldMs < MAX_POSE_PREDICTION_MS ? heldMs : MAX_POSE_PREDICTION_MS;
  const float predX = poseX + poseVX * predMs / 1000.0f;
  const float predY = poseY + poseVY * predMs / 1000.0f;

  Serial.print('P');
  Serial.print(' ');
  Serial.print(predX, 1);
  Serial.print(' ');
  Serial.print(predY, 1);
  Serial.print(' ');
  Serial.print(YAW_SIGN * currentYawDeg, 2);
  Serial.print(' ');
  Serial.println(poseQuality, 2);
}

void setup() {
  Serial.begin(115200);
  const unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}  // give the PC a moment to open the port

  Wire.begin();
  buildFieldMap();
  initIMU();
  initLidar();

  Serial.println(F("# LD14P lidar/pose test stream ready"));
  Serial.println(F("# P x y heading_deg quality | binary lidar-hit frames (see header comment)"));
}

void loop() {
  updateIMU();

  unsigned long now = millis();
  pruneRays(now);
  consumeLidar();
  now = millis();
  pruneRays(now);
  updateLidarSpeedController(now);
  reportLidarStatus(now);

  if (isIMUHeadingFresh() && newRayCount >= NEW_RAY_TRIGGER && now - lastFitMs >= FIT_INTERVAL_MS) {
    lastFitMs = now;
    newRayCount = 0;
    updatePoseFromLidar();
  }

  if (poseInitialized && now - lastPoseFixMs > POSE_VEL_TIMEOUT_MS) {
    // Lost lock for a while -- stop dead-reckoning on a stale velocity estimate.
    poseVX = 0.0f;
    poseVY = 0.0f;
  }

  if (now - lastPoseStreamMs >= POSE_STREAM_INTERVAL_MS) {
    lastPoseStreamMs = now;
    streamPose();
  }
}
