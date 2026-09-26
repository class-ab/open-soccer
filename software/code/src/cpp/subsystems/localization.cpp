#include "include/subsystems/localization.h"

#include <Arduino.h>
#include <math.h>

#include "include/subsystems/communication.h"
#include "include/subsystems/imu.h"
#include "include/subsystems/robot_config.h"
#include "include/subsystems/robot_state.h"

namespace {
constexpr uint8_t PACKET_SIZE = 47;
constexpr uint8_t PACKET_HEADER = 0x54;
constexpr uint8_t PACKET_VERLEN = 0x2C;
constexpr uint8_t POINTS_PER_PACKET = 12;
constexpr size_t LIDAR_RX_BUFFER_SIZE = 16384;

constexpr float FIELD_WIDTH_MM = 2430.0f;
constexpr float FIELD_HEIGHT_MM = 1820.0f;
constexpr float NOTCH_DEPTH_MM = 226.0f;
constexpr float NOTCH_HEIGHT_MM = 470.0f;
constexpr float MIN_RANGE_MM = 50.0f;
constexpr float MAX_RANGE_MM = 12000.0f;
constexpr float LOCAL_DEG_TO_RAD = PI / 180.0f;
constexpr float HUBER_MM = 120.0f;
constexpr float MAX_ACCEPTED_COST = 24000.0f;
constexpr float MAX_FIX_CORRECTION_MM = 35.0f;
constexpr float FIT_CONDITION_MIN = 0.004f;
constexpr uint8_t MIN_FIT_RAYS = 18;
constexpr uint8_t MAX_FIT_RAYS = 72;
// Higher = fewer, more-independent fits (less jitter, same latency floor set by RAY_WINDOW_MS).
constexpr uint8_t NEW_RAY_TRIGGER = 6;
constexpr uint16_t MAX_WINDOW_RAYS = 512;
constexpr unsigned long RAY_WINDOW_MS = 35;
constexpr unsigned long FIT_INTERVAL_MS = 10;
constexpr unsigned long POSE_TIMEOUT_MS = 1200;
constexpr unsigned long OPPONENT_TIMEOUT_MS = 500;
constexpr unsigned long LIDAR_INTERBYTE_TIMEOUT_MS = 25;

// Pose is just the last accepted lidar fit -- no velocity/dead-reckoning model.
// MAX_FIX_CORRECTION_MM above is the only guard, rejecting single outlier fits.

constexpr float POLE_ANGLES_DEG[4] = {-135.0f, -45.0f, 45.0f, 135.0f};
constexpr float POLE_HALF_WIDTH_DEG = 3.4f;
constexpr uint8_t MAX_OPPONENTS = 3;
constexpr float OPPONENT_CLUSTER_GAP_MM = 120.0f;
constexpr float OPPONENT_MIN_RADIUS_MM = 35.0f;
constexpr float OPPONENT_MAX_RADIUS_MM = 150.0f;
constexpr uint8_t OPPONENT_MIN_POINTS = 5;

struct LidarPacket {
  uint16_t speed;
  uint16_t startAngle;
  uint16_t distanceMm[POINTS_PER_PACKET];
  uint8_t intensity[POINTS_PER_PACKET];
  uint16_t endAngle;
};

struct Ray {
  float dx;
  float dy;
  float range;
  float weight;
  float invDx;  // 1/dx, 1/dy cached at append time -- fixed for the ray's lifetime
  float invDy;
  unsigned long timestampMs;
};

struct MapEdge {
  float ax;
  float ay;
  float ex;
  float ey;
};

struct RayHit {
  float range;
  bool vertical;
};

struct FitResult {
  float cost;
  float hxx;
  float hxy;
  float hyy;
  float gx;
  float gy;
  float weightSum;
  uint16_t hitCount;
};

constexpr float mapX[12] = {
    0.0f, 0.0f, NOTCH_DEPTH_MM, NOTCH_DEPTH_MM, 0.0f, 0.0f,
    FIELD_WIDTH_MM, FIELD_WIDTH_MM,
    FIELD_WIDTH_MM - NOTCH_DEPTH_MM, FIELD_WIDTH_MM - NOTCH_DEPTH_MM,
    FIELD_WIDTH_MM, FIELD_WIDTH_MM};
constexpr float mapY[12] = {
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

MapEdge mapEdges[12];
Ray rays[MAX_WINDOW_RAYS];
uint16_t rayStart = 0;
uint16_t rayCount = 0;
uint16_t newRayCount = 0;
uint8_t lidarRxStorage[LIDAR_RX_BUFFER_SIZE];
uint8_t packetBuffer[PACKET_SIZE];
uint8_t packetIndex = 0;
enum ReceiveState : uint8_t { WAIT_HEADER, WAIT_LENGTH, READ_PACKET };
ReceiveState receiveState = WAIT_HEADER;

RobotPose robotPose = {false, 0.0f, 0.0f, 0.0f, 0.0f, 0};
FieldBall fieldBall = {false, 0.0f, 0.0f, 0.0f, 0.0f, 0};
OpponentRobot opponents[MAX_OPPONENTS] = {};
uint8_t opponentCount = 0;
unsigned long opponentTimestampMs = 0;

bool poseInitialized = false;
float poseX = FIELD_WIDTH_MM * 0.5f;
float poseY = FIELD_HEIGHT_MM * 0.5f;
float poseQuality = 0.0f;
unsigned long lastPoseFixMs = 0;  // timestamp of last accepted fit
unsigned long lastFitMs = 0;      // gates fit *attempt* cadence
unsigned long lastLidarByteMs = 0;
unsigned long lastValidLidarPacketMs = 0;
uint32_t validLidarPackets = 0;
uint32_t invalidLidarPackets = 0;

// LD14P motor speed PWM controller bookkeeping.
uint16_t lastLidarSpeedDegS = 0;
unsigned long lastLidarPacketMs = 0;
bool lidarPacketSeen = false;
uint32_t lidarSpeedSumDegS = 0;
uint16_t lidarSpeedSampleCount = 0;
float lidarPwmDutyPercent = LIDAR_PWM_ENTRY_DUTY_PERCENT;
unsigned long lastLidarControlMs = 0;

#ifdef LIDAR_POSE_STREAM
unsigned long lastPoseStreamMs = 0;
#endif

float wrappedDifference(float a, float b) {
  return fmodf(a - b + 540.0f, 360.0f) - 180.0f;
}

uint16_t readU16(const uint8_t *bytes) {
  return static_cast<uint16_t>(bytes[0]) |
         (static_cast<uint16_t>(bytes[1]) << 8);
}

uint8_t lidarCrc8(const uint8_t *bytes, uint8_t length) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < length; ++i) {
    crc ^= bytes[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x4D)
                         : static_cast<uint8_t>(crc << 1);
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

bool insideField(float x, float y) {
  if (x < 0.0f || x > FIELD_WIDTH_MM ||
      y < 0.0f || y > FIELD_HEIGHT_MM) {
    return false;
  }
  const float low = FIELD_HEIGHT_MM * 0.5f - NOTCH_HEIGHT_MM * 0.5f;
  const float high = FIELD_HEIGHT_MM * 0.5f + NOTCH_HEIGHT_MM * 0.5f;
  return !(y > low && y < high &&
           (x < NOTCH_DEPTH_MM || x > FIELD_WIDTH_MM - NOTCH_DEPTH_MM));
}

bool angleBlocked(float localAngleDeg) {
  for (float poleAngle : POLE_ANGLES_DEG) {
    if (fabsf(wrappedDifference(localAngleDeg, poleAngle)) <=
        POLE_HALF_WIDTH_DEG) {
      return true;
    }
  }
  return false;
}

float pointWeight(uint16_t range, uint8_t intensity) {
  if (range < MIN_RANGE_MM || range > MAX_RANGE_MM || intensity < 8) {
    return 0.0f;
  }
  return intensity / 255.0f;
}

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

Ray &rayAt(uint16_t logicalIndex) {
  return rays[(rayStart + logicalIndex) % MAX_WINDOW_RAYS];
}

void pruneRays(unsigned long now) {
  while (rayCount > 0 &&
         now - rayAt(0).timestampMs > RAY_WINDOW_MS) {
    rayStart = (rayStart + 1) % MAX_WINDOW_RAYS;
    --rayCount;
  }
}

bool rayToField(float x, float y, float dx, float dy, RayHit &hit) {
  float nearest = MAX_RANGE_MM;
  bool found = false;
  bool nearestVertical = false;

  for (const MapEdge &edge : mapEdges) {
    const float denominator = dx * edge.ey - dy * edge.ex;
    if (fabsf(denominator) < 1e-6f) continue;

    const float ax = edge.ax - x;
    const float ay = edge.ay - y;
    const float distance = (ax * edge.ey - ay * edge.ex) / denominator;
    const float alongEdge = (ax * dy - ay * dx) / denominator;
    if (distance > 0.0f && alongEdge >= 0.0f && alongEdge <= 1.0f &&
        distance < nearest) {
      nearest = distance;
      nearestVertical = fabsf(edge.ex) < 1e-5f;
      found = true;
    }
  }

  if (found) {
    hit = {nearest, nearestVertical};
  }
  return found;
}

float robustLoss(float residual) {
  const float absolute = fabsf(residual);
  return absolute <= HUBER_MM
             ? residual * residual
             : 2.0f * HUBER_MM * absolute - HUBER_MM * HUBER_MM;
}

FitResult evaluatePose(float x, float y, const Ray *samples, uint8_t count,
                       float totalWeight, bool buildHessian) {
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
      const float robustWeight =
          ray.weight * (fabsf(residual) > HUBER_MM
                            ? HUBER_MM / fabsf(residual)
                            : 1.0f);
      const float jx = hit.vertical ? ray.invDx : 0.0f;
      const float jy = hit.vertical ? 0.0f : ray.invDy;
      fit.hxx += robustWeight * jx * jx;
      fit.hxy += robustWeight * jx * jy;
      fit.hyy += robustWeight * jy * jy;
      fit.gx += robustWeight * jx * residual;
      fit.gy += robustWeight * jy * residual;
    }
  }

  fit.cost = fit.weightSum > 0.0f ? weightedCost / fit.weightSum : 1e30f;
  return fit;
}

bool wellConditioned(const FitResult &fit) {
  const float trace = fit.hxx + fit.hyy;
  if (trace <= 1e-6f || fit.hitCount < MIN_FIT_RAYS) return false;
  const float determinant = fit.hxx * fit.hyy - fit.hxy * fit.hxy;
  return determinant > FIT_CONDITION_MIN * trace * trace;
}

uint8_t collectFitRays(Ray *samples) {
  if (rayCount < MIN_FIT_RAYS) return 0;
  const uint8_t count = rayCount < MAX_FIT_RAYS
                            ? static_cast<uint8_t>(rayCount)
                            : MAX_FIT_RAYS;
  for (uint8_t i = 0; i < count; ++i) {
    const uint16_t source = static_cast<uint16_t>(
        (static_cast<uint32_t>(i) * rayCount) / count);
    samples[i] = rayAt(source);
  }
  return count;
}

bool refinePose(float &x, float &y, const Ray *samples, uint8_t count,
                float totalWeight, FitResult &result) {
  for (uint8_t iteration = 0; iteration < 5; ++iteration) {
    const FitResult current = evaluatePose(x, y, samples, count, totalWeight, true);
    if (!wellConditioned(current)) return false;

    const float determinant = current.hxx * current.hyy -
                              current.hxy * current.hxy;
    float stepX = -(current.hyy * current.gx -
                    current.hxy * current.gy) / determinant;
    float stepY = -(-current.hxy * current.gx +
                    current.hxx * current.gy) / determinant;

    const float stepLength = hypotf(stepX, stepY);
    if (stepLength > 160.0f) {
      const float scale = 160.0f / stepLength;
      stepX *= scale;
      stepY *= scale;
    }
    if (hypotf(stepX, stepY) < 0.5f) break;

    bool improved = false;
    float scale = 1.0f;
    for (uint8_t trial = 0; trial < 5; ++trial) {
      const float candidateX = x + stepX * scale;
      const float candidateY = y + stepY * scale;
      if (insideField(candidateX, candidateY)) {
        const FitResult candidate =
            evaluatePose(candidateX, candidateY, samples, count, totalWeight, false);
        if (candidate.cost <= current.cost) {
          x = candidateX;
          y = candidateY;
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

bool coarseSearch(float centerX, float centerY, float halfRangeX,
                  float halfRangeY, float step, const Ray *samples,
                  uint8_t count, float totalWeight, float &bestX, float &bestY) {
  float bestCost = 1e30f;
  bool found = false;
  for (float x = centerX - halfRangeX; x <= centerX + halfRangeX; x += step) {
    for (float y = centerY - halfRangeY; y <= centerY + halfRangeY; y += step) {
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

void finishOpponentCluster(float sumX, float sumY, float sumSquared,
                           uint16_t count, unsigned long now) {
  if (count < OPPONENT_MIN_POINTS || opponentCount >= MAX_OPPONENTS) return;
  const float centerX = sumX / count;
  const float centerY = sumY / count;
  const float radius = sqrtf(fmaxf(
      0.0f, sumSquared / count - centerX * centerX - centerY * centerY));
  if (radius < OPPONENT_MIN_RADIUS_MM || radius > OPPONENT_MAX_RADIUS_MM ||
      !insideField(centerX, centerY)) {
    return;
  }

  opponents[opponentCount++] = {
      true, centerX - FIELD_WIDTH_MM * 0.5f,
      centerY - FIELD_HEIGHT_MM * 0.5f,
      fminf(1.0f, count / 10.0f), now};
}

void updateOpponentDetections(unsigned long now) {
  opponentCount = 0;
  if (!poseInitialized) return;

  float sumX = 0.0f;
  float sumY = 0.0f;
  float sumSquared = 0.0f;
  float previousX = 0.0f;
  float previousY = 0.0f;
  uint16_t clusterSize = 0;

  for (uint16_t i = 0; i < rayCount; ++i) {
    const Ray &ray = rayAt(i);
    const float pointX = poseX + ray.dx * ray.range;
    const float pointY = poseY + ray.dy * ray.range;
    RayHit hit;
    const bool objectReturn = insideField(pointX, pointY) &&
        rayToField(poseX, poseY, ray.dx, ray.dy, hit) &&
        ray.range < hit.range - 100.0f;

    if (!objectReturn) {
      finishOpponentCluster(sumX, sumY, sumSquared, clusterSize, now);
      sumX = sumY = sumSquared = 0.0f;
      clusterSize = 0;
      continue;
    }

    const float gap = clusterSize == 0
                          ? 0.0f
                          : hypotf(pointX - previousX, pointY - previousY);
    if (clusterSize > 0 && gap > OPPONENT_CLUSTER_GAP_MM) {
      finishOpponentCluster(sumX, sumY, sumSquared, clusterSize, now);
      sumX = sumY = sumSquared = 0.0f;
      clusterSize = 0;
    }
    sumX += pointX;
    sumY += pointY;
    sumSquared += pointX * pointX + pointY * pointY;
    previousX = pointX;
    previousY = pointY;
    ++clusterSize;
  }

  finishOpponentCluster(sumX, sumY, sumSquared, clusterSize, now);
  opponentTimestampMs = now;
}

void updatePoseFromLidar(unsigned long now) {
  Ray samples[MAX_FIT_RAYS];
  const uint8_t count = collectFitRays(samples);
  if (count < MIN_FIT_RAYS) return;
  float totalWeight = 0.0f;
  for (uint8_t i = 0; i < count; ++i) totalWeight += samples[i].weight;

  // Search center is just the last accepted fit -- no velocity extrapolation,
  // so a fit reflects only what the sensor sees right now.
  float candidateX = poseX;
  float candidateY = poseY;
  FitResult fit = {};

  if (!poseInitialized) {
    if (!coarseSearch(FIELD_WIDTH_MM * 0.5f, FIELD_HEIGHT_MM * 0.5f,
                      FIELD_WIDTH_MM * 0.5f, FIELD_HEIGHT_MM * 0.5f,
                      100.0f, samples, count, totalWeight, candidateX, candidateY)) {
      return;
    }
    if (!refinePose(candidateX, candidateY, samples, count, totalWeight, fit)) return;
  } else if (!refinePose(candidateX, candidateY, samples, count, totalWeight, fit) ||
             fit.cost > MAX_ACCEPTED_COST) {
    if (!coarseSearch(poseX, poseY, 300.0f, 300.0f, 75.0f,
                      samples, count, totalWeight, candidateX, candidateY) ||
        !refinePose(candidateX, candidateY, samples, count, totalWeight, fit)) {
      return;
    }
  }

  if (fit.cost > MAX_ACCEPTED_COST) return;
  const float quality = 1.0f / (1.0f + fit.cost / 25000.0f);
  if (!poseInitialized) {
    poseX = candidateX;
    poseY = candidateY;
    poseInitialized = true;
    poseQuality = quality;
    lastPoseFixMs = now;
    updateOpponentDetections(now);
    return;
  }

  float correctionX = candidateX - poseX;
  float correctionY = candidateY - poseY;
  const float correction = hypotf(correctionX, correctionY);
  if (correction > MAX_FIX_CORRECTION_MM) {
    // Disagrees sharply with the last fix -- clamp like an outlier, don't chase it.
    const float scale = MAX_FIX_CORRECTION_MM / correction;
    correctionX *= scale;
    correctionY *= scale;
  }

  poseX += correctionX;
  poseY += correctionY;
  poseQuality += 0.25f * (quality - poseQuality);

  lastPoseFixMs = now;
  updateOpponentDetections(now);
}

#ifdef LIDAR_POSE_STREAM
// Sends every valid point from one lidar packet as a single binary frame
// instead of one Serial.print per point, to keep formatting overhead off
// the localization hot path.
//   byte 0        : 0xAA sync byte
//   byte 1        : point count N (0-12)
//   N * 4 bytes   : (int16 x_mm, int16 y_mm) little-endian, world frame
//   byte last     : XOR checksum of count byte + all payload bytes
constexpr uint8_t BINARY_FRAME_SYNC = 0xAA;
constexpr unsigned long POSE_STREAM_INTERVAL_MS = 20;

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

// "P <x_mm> <y_mm> <heading_deg> <quality 0-1>\n" -- the raw last-fit pose, no extrapolation.
void streamPose(unsigned long now) {
  (void)now;
  Serial.print('P');
  Serial.print(' ');
  Serial.print(poseX, 1);
  Serial.print(' ');
  Serial.print(poseY, 1);
  Serial.print(' ');
  Serial.print(YAW_SIGN * currentYawDeg, 2);
  Serial.print(' ');
  Serial.println(poseQuality, 2);
}
#endif  // LIDAR_POSE_STREAM

void addPacket(const LidarPacket &packet) {
  const int32_t angleSpan =
      (static_cast<int32_t>(packet.endAngle) - packet.startAngle + 36000) % 36000;
  const float headingDeg = YAW_SIGN * currentYawDeg;
  const float lidarSweepMs =
      packet.speed > 0 ? angleSpan * 10.0f / packet.speed : 0.0f;
  const float serialDelayMs =
      PACKET_SIZE * 10000.0f / LIDAR_UART_BAUD;
  const unsigned long packetEndMs = millis();

  lastLidarSpeedDegS = packet.speed;
  lastLidarPacketMs = packetEndMs;
  lidarPacketSeen = true;
  if (packet.speed > 0) {
    lidarSpeedSumDegS += packet.speed;
    if (lidarSpeedSampleCount < UINT16_MAX) ++lidarSpeedSampleCount;
  }

#ifdef LIDAR_POSE_STREAM
  int16_t batchCoords[POINTS_PER_PACKET * 2];
  uint8_t batchCount = 0;
#endif

  for (uint8_t i = 0; i < POINTS_PER_PACKET; ++i) {
    const int32_t angleHundredths =
        (packet.startAngle + angleSpan * i / (POINTS_PER_PACKET - 1)) % 36000;
    const float localAngle = angleHundredths * 0.01f;
    if (angleBlocked(localAngle)) continue;

    const float weight = pointWeight(packet.distanceMm[i], packet.intensity[i]);
    if (weight <= 0.0f) continue;

    // LD14P bearings increase clockwise; field coordinates increase CCW.
    const float fraction = i / static_cast<float>(POINTS_PER_PACKET - 1);
    const float ageMs = serialDelayMs * 0.5f +
                        lidarSweepMs * (1.0f - fraction);
    const float pointHeading =
        headingDeg - YAW_SIGN * getIMUYawRateDegPerSec() * (ageMs / 1000.0f);
    const float worldRad =
        (pointHeading - localAngle) * LOCAL_DEG_TO_RAD;
    const float dx = cosf(worldRad);
    const float dy = sinf(worldRad);
    const float range = static_cast<float>(packet.distanceMm[i]);

    appendRay({dx, dy, range, weight, 1.0f / dx, 1.0f / dy,
               packetEndMs - static_cast<unsigned long>(ageMs)});

#ifdef LIDAR_POSE_STREAM
    if (batchCount < POINTS_PER_PACKET) {
      batchCoords[batchCount * 2] = static_cast<int16_t>(lroundf(poseX + range * dx));
      batchCoords[batchCount * 2 + 1] = static_cast<int16_t>(lroundf(poseY + range * dy));
      ++batchCount;
    }
#endif
  }

#ifdef LIDAR_POSE_STREAM
  streamPointsBatch(batchCoords, batchCount);
#endif
}

void writeLidarPwm(float dutyPercent) {
  // analogWriteResolution() is board-wide on Teensy, not per-pin -- stay on the
  // default 8-bit range so this doesn't reinterpret drivebase's PWM duty cycles.
  const uint16_t pwmMax = (1U << 8) - 1;
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
    return;
  }

  const float magnitude = fminf(3.0f, fmaxf(0.05f, fabsf(error) / 1000.0f));
  const float direction = error > 0.0f ? 1.0f : -1.0f;
  const float requestedDuty = lidarPwmDutyPercent + direction * magnitude;
  const float boundedDuty = fminf(LIDAR_PWM_MAX_DUTY_PERCENT,
                                  fmaxf(LIDAR_PWM_MIN_DUTY_PERCENT, requestedDuty));

  if (fabsf(boundedDuty - lidarPwmDutyPercent) < 0.001f) {
    return;
  }

  lidarPwmDutyPercent = boundedDuty;
  writeLidarPwm(lidarPwmDutyPercent);
}

#ifdef DEBUG_LIDAR
void reportLidarStatus(unsigned long now) {
  static unsigned long lastLidarDebugMs = 0;
  if (now - lastLidarDebugMs < 1000) return;
  lastLidarDebugMs = now;

  Serial.print("LIDAR pwmDuty=");
  Serial.print(lidarPwmDutyPercent, 2);
  Serial.print(" speedHz=");
  Serial.print(lastLidarSpeedDegS / 360.0f, 2);
  Serial.print(" packetAge=");
  Serial.print(lidarPacketSeen ? now - lastLidarPacketMs : 0);
  Serial.print("ms bytesAge=");
  Serial.print(lastLidarByteMs == 0 ? 0 : now - lastLidarByteMs);
  Serial.print("ms rays=");
  Serial.print(rayCount);
  Serial.print(" fixAge=");
  Serial.print(lastPoseFixMs == 0 ? 0 : now - lastPoseFixMs);
  Serial.print("ms imuFresh=");
  Serial.print(isIMUHeadingFresh());
  Serial.print(" good/bad=");
  Serial.print(validLidarPackets);
  Serial.print('/');
  Serial.println(invalidLidarPackets);
}
#endif  // DEBUG_LIDAR

void consumeLidar() {
  while (LIDAR_UART.available() > 0) {
    const int incoming = LIDAR_UART.read();
    if (incoming < 0) break;
    const uint8_t value = static_cast<uint8_t>(incoming);
    const unsigned long byteNow = millis();

    // Discard partial frames after the scanner pauses or the UART drops data.
    // A normal 47-byte frame arrives far faster than this timeout at 230400 baud.
    if (packetIndex > 0 && byteNow - lastLidarByteMs >
                               LIDAR_INTERBYTE_TIMEOUT_MS) {
      packetIndex = 0;
      receiveState = WAIT_HEADER;
    }
    lastLidarByteMs = byteNow;

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
          if (parsePacket(packetBuffer, packet)) {
            lastValidLidarPacketMs = millis();
            ++validLidarPackets;
            if (isIMUHeadingFresh()) addPacket(packet);
          } else {
            ++invalidLidarPackets;
          }
          packetIndex = 0;
          receiveState = WAIT_HEADER;
        }
        break;
    }
  }
}

void updatePublicPose(unsigned long now) {
  const unsigned long fixAge = poseInitialized ? now - lastPoseFixMs
                                               : POSE_TIMEOUT_MS + 1;
  const float freshness = fixAge < POSE_TIMEOUT_MS
                              ? 1.0f - fixAge /
                                            static_cast<float>(POSE_TIMEOUT_MS)
                              : 0.0f;
  robotPose = {
      poseInitialized && fixAge <= POSE_TIMEOUT_MS && isIMUHeadingFresh(),
      poseX - FIELD_WIDTH_MM * 0.5f,
      poseY - FIELD_HEIGHT_MM * 0.5f,
      YAW_SIGN * currentYawDeg,
      poseQuality * freshness,
      now};
}

void updateFieldBall(unsigned long now) {
  const BallPacket ball = latestBallPacket;
  const bool poseFresh = robotPose.valid;
  const bool ballFresh = ball.detected &&
                         now - lastBallPacketMs <= BALL_DATA_TIMEOUT_MS;
  if (!poseFresh || !ballFresh) {
    fieldBall.valid = false;
    return;
  }

  const float angleDeg = ball.angleDeg + CAMERA_MOUNT_OFFSET_DEG;
  const float worldAngle =
      (robotPose.headingDeg - angleDeg) * LOCAL_DEG_TO_RAD;
  const float distanceMm = ball.distanceCM * 10.0f;
  fieldBall = {true,
               robotPose.xMm + distanceMm * cosf(worldAngle),
               robotPose.yMm + distanceMm * sinf(worldAngle),
               ball.distanceCM, angleDeg, lastBallPacketMs};
}
}  // namespace

void initLocalization() {
  LIDAR_UART.addMemoryForRead(lidarRxStorage, sizeof(lidarRxStorage));
  LIDAR_UART.begin(LIDAR_UART_BAUD);
  analogWriteFrequency(LIDAR_SPEED_CONTROL_PIN, LIDAR_PWM_FREQUENCY_HZ);
  lidarPwmDutyPercent = LIDAR_PWM_ENTRY_DUTY_PERCENT;
  writeLidarPwm(lidarPwmDutyPercent);

  for (uint8_t i = 0; i < 12; ++i) {
    const uint8_t next = (i + 1) % 12;
    mapEdges[i] = {mapX[i], mapY[i],
                   mapX[next] - mapX[i], mapY[next] - mapY[i]};
  }
  rayStart = rayCount = newRayCount = 0;
  poseInitialized = false;
  poseX = FIELD_WIDTH_MM * 0.5f;
  poseY = FIELD_HEIGHT_MM * 0.5f;
  poseQuality = 0.0f;
  lastPoseFixMs = 0;
  lastFitMs = 0;
  lastLidarByteMs = 0;
  lastValidLidarPacketMs = 0;
  validLidarPackets = 0;
  invalidLidarPackets = 0;
  lastLidarSpeedDegS = 0;
  lastLidarPacketMs = 0;
  lidarPacketSeen = false;
  lidarSpeedSumDegS = 0;
  lidarSpeedSampleCount = 0;
  lastLidarControlMs = millis();
  opponentCount = 0;
  opponentTimestampMs = 0;
  robotPose = {false, 0.0f, 0.0f, 0.0f, 0.0f, 0};
  fieldBall = {false, 0.0f, 0.0f, 0.0f, 0.0f, 0};
  Serial.println("LD14P rolling localization ready");
}

void updateLocalization() {
  unsigned long now = millis();
  pruneRays(now);
  consumeLidar();
  now = millis();
  pruneRays(now);
  updateLidarSpeedController(now);

  if (isIMUHeadingFresh() && newRayCount >= NEW_RAY_TRIGGER &&
      now - lastFitMs >= FIT_INTERVAL_MS) {
    lastFitMs = now;
    newRayCount = 0;
    updatePoseFromLidar(now);
  }

  updatePublicPose(now);
  updateFieldBall(now);

#ifdef LIDAR_POSE_STREAM
  if (now - lastPoseStreamMs >= POSE_STREAM_INTERVAL_MS) {
    lastPoseStreamMs = now;
    streamPose(now);
  }
#endif

#ifdef DEBUG_LIDAR
  reportLidarStatus(now);
#endif
}

void getRobotPose(RobotPose &out) {
  out = robotPose;
}

void getFieldBall(FieldBall &out) {
  out = fieldBall;
}

void getOpponents(OpponentRobot *out, int maxOpponents, int &count) {
  count = 0;
  if (out == nullptr || maxOpponents <= 0 ||
      millis() - opponentTimestampMs > OPPONENT_TIMEOUT_MS) {
    return;
  }
  const int available = opponentCount < maxOpponents ? opponentCount : maxOpponents;
  for (int i = 0; i < available; ++i) out[count++] = opponents[i];
}

