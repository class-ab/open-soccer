#include "include/subsystems/localization.h"

#include <Arduino.h>
#include <math.h>

#include "include/subsystems/communication.h"
#include "include/subsystems/imu.h"
#include "include/subsystems/robot_config.h"
#include "include/subsystems/robot_state.h"

namespace {
// LD14P packet: header, length, speed, start angle, 12 x (range,intensity),
// end angle, timestamp, CRC.
constexpr uint8_t PACKET_SIZE = 47;
constexpr uint8_t PACKET_HEADER = 0x54;
constexpr uint8_t PACKET_VERLEN = 0x2C;
constexpr int POINTS_PER_PACKET = 12;
constexpr size_t LIDAR_RX_BUFFER_SIZE = 8192;

constexpr float FIELD_WIDTH_MM = 2430.0f;
constexpr float FIELD_HEIGHT_MM = 1820.0f;
constexpr float NOTCH_DEPTH_MM = 226.0f;
constexpr float NOTCH_HEIGHT_MM = 470.0f;
constexpr float MIN_RANGE_MM = 50.0f;
constexpr float MAX_RANGE_MM = 12000.0f;
constexpr float LOCAL_DEG_TO_RAD = PI / 180.0f;

constexpr int BIN_COUNT = 72;  // 5-degree world-frame rays
constexpr int MIN_VALID_BINS = 18;
constexpr unsigned long POSE_TIMEOUT_MS = 1500;
constexpr unsigned long OPPONENT_TIMEOUT_MS = 500;

// Chassis occlusion angles are measured in the LiDAR's local frame.
constexpr float POLE_ANGLES_DEG[4] = {-135.0f, -45.0f, 45.0f, 135.0f};
constexpr float POLE_HALF_WIDTH_DEG = 3.4f;

constexpr int MAX_SCAN_POINTS = 864;
constexpr int MAX_OPPONENTS = 3;
constexpr float OPPONENT_CLUSTER_GAP_MM = 120.0f;
constexpr float OPPONENT_MIN_RADIUS_MM = 35.0f;
constexpr float OPPONENT_MAX_RADIUS_MM = 150.0f;
constexpr int OPPONENT_MIN_POINTS = 5;

struct LidarPacket {
  uint16_t speed;
  uint16_t startAngle;
  uint16_t distanceMm[POINTS_PER_PACKET];
  uint8_t intensity[POINTS_PER_PACKET];
  uint16_t endAngle;
};

struct RayBin {
  float weightedRange;
  float weight;
  float weightedDirX;
  float weightedDirY;
};

struct Candidate {
  float x;
  float y;
  float cost;
};

struct ScanPoint {
  float localX;
  float localY;
  float range;
  float weight;
  float headingDeg;
};

struct MapEdge {
  float ax;
  float ay;
  float ex;
  float ey;
};

constexpr float mapX[12] = {
  0.0f, 0.0f, NOTCH_DEPTH_MM, NOTCH_DEPTH_MM, 0.0f, 0.0f,
  FIELD_WIDTH_MM, FIELD_WIDTH_MM,
  FIELD_WIDTH_MM - NOTCH_DEPTH_MM, FIELD_WIDTH_MM - NOTCH_DEPTH_MM,
  FIELD_WIDTH_MM, FIELD_WIDTH_MM
};
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
  FIELD_HEIGHT_MM
};

RayBin scanBins[BIN_COUNT];
ScanPoint scanPoints[MAX_SCAN_POINTS];
uint16_t scanPointCount = 0;
MapEdge mapEdges[12];
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
float fusedX = FIELD_WIDTH_MM * 0.5f;
float fusedY = FIELD_HEIGHT_MM * 0.5f;
int32_t lastPacketStartAngle = -1;

#ifdef DEBUG_LIDAR
unsigned long debugBytes = 0;
unsigned long debugPackets = 0;
unsigned long debugBadPackets = 0;
unsigned long debugFixes = 0;
unsigned long debugPrintMs = 0;
int debugRayCount = 0;
#endif

float wrap360(float angle) {
  angle = fmodf(angle, 360.0f);
  return angle < 0.0f ? angle + 360.0f : angle;
}

float wrappedDifference(float a, float b) {
  float difference = fmodf(a - b + 540.0f, 360.0f) - 180.0f;
  return difference;
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
  for (int i = 0; i < POINTS_PER_PACKET; ++i) {
    const uint8_t *point = bytes + 6 + i * 3;
    packet.distanceMm[i] = readU16(point);
    packet.intensity[i] = point[2];
  }
  packet.endAngle = readU16(bytes + 42);
  return true;
}

void clearScan() {
  for (int i = 0; i < BIN_COUNT; ++i) {
    scanBins[i] = {0.0f, 0.0f, 0.0f, 0.0f};
  }
  scanPointCount = 0;
}

bool insideField(float x, float y) {
  if (x < 0.0f || x > FIELD_WIDTH_MM || y < 0.0f || y > FIELD_HEIGHT_MM) {
    return false;
  }
  const float low = FIELD_HEIGHT_MM * 0.5f - NOTCH_HEIGHT_MM * 0.5f;
  const float high = FIELD_HEIGHT_MM * 0.5f + NOTCH_HEIGHT_MM * 0.5f;
  return !(y > low && y < high &&
           (x < NOTCH_DEPTH_MM || x > FIELD_WIDTH_MM - NOTCH_DEPTH_MM));
}

float rayToField(float x, float y, float dx, float dy) {
  float nearest = MAX_RANGE_MM;
  bool found = false;
  for (int i = 0; i < 12; ++i) {
    const MapEdge &edge = mapEdges[i];
    const float denominator = dx * edge.ey - dy * edge.ex;
    if (fabsf(denominator) < 1e-6f) continue;

    const float ax = edge.ax - x;
    const float ay = edge.ay - y;
    const float t = (ax * edge.ey - ay * edge.ex) / denominator;
    const float u = (ax * dy - ay * dx) / denominator;
    if (t > 0.0f && u >= 0.0f && u <= 1.0f && t < nearest) {
      nearest = t;
      found = true;
    }
  }
  return found ? nearest : -1.0f;
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
  const float confidence = intensity / 255.0f;
  const float midRange = 0.35f * (MIN_RANGE_MM + MAX_RANGE_MM);
  const float spread = MAX_RANGE_MM * 0.6f;
  const float rangeWeight = fmaxf(0.05f, 1.0f -
      fabsf(static_cast<float>(range) - midRange) / spread);
  return confidence * rangeWeight;
}

void emitPose() {
  Serial.print("POSE,1,");
  Serial.print(robotPose.xMm, 1);
  Serial.print(',');
  Serial.print(robotPose.yMm, 1);
  Serial.print(',');
  Serial.print(robotPose.headingDeg, 1);
  Serial.print(',');
  Serial.print(robotPose.quality, 3);
  Serial.print(',');
  Serial.println(robotPose.timestampMs);
}

float robustResidualCost(float residual) {
  // Huber loss limits the effect of multipath and moving obstacles.
  constexpr float HUBER_MM = 120.0f;
  const float absolute = fabsf(residual);
  return absolute <= HUBER_MM ? residual * residual
                              : 2.0f * HUBER_MM * absolute -
                                    HUBER_MM * HUBER_MM;
}

Candidate searchCandidates(float centerX, float centerY, float halfRange,
                           float step, const float *ranges,
                           const float *weights, const float *dirX,
                           const float *dirY, int rayCount) {
  Candidate best = {centerX, centerY, 1e30f};
  const int steps = static_cast<int>(2.0f * halfRange / step + 0.5f);
  for (int ix = 0; ix <= steps; ++ix) {
    const float x = centerX - halfRange + ix * step;
    for (int iy = 0; iy <= steps; ++iy) {
      const float y = centerY - halfRange + iy * step;
      if (!insideField(x, y)) continue;

      float cost = 0.0f;
      float weightSum = 0.0f;
      for (int ray = 0; ray < rayCount; ++ray) {
        const float expected = rayToField(x, y, dirX[ray], dirY[ray]);
        if (expected < 0.0f) continue;
        cost += weights[ray] * robustResidualCost(ranges[ray] - expected);
        weightSum += weights[ray];
      }
      if (weightSum > 0.0f) {
        const float meanCost = cost / weightSum;
        if (meanCost < best.cost) best = {x, y, meanCost};
      }
    }
  }
  return best;
}

void collectWorldRays(float *ranges, float *weights, float *dirX,
                      float *dirY, int &count) {
  count = 0;
  for (int i = 0; i < BIN_COUNT; ++i) {
    const RayBin &bin = scanBins[i];
    if (bin.weight <= 0.0f) continue;
    const float directionLength = sqrtf(bin.weightedDirX * bin.weightedDirX +
                                        bin.weightedDirY * bin.weightedDirY);
    if (directionLength < 1e-5f) continue;
    ranges[count] = bin.weightedRange / bin.weight;
    weights[count] = bin.weight;
    dirX[count] = bin.weightedDirX / directionLength;
    dirY[count] = bin.weightedDirY / directionLength;
    ++count;
  }
}

void updateOpponentDetections(float poseX, float poseY) {
  opponentCount = 0;
  if (!robotPose.valid) return;

  // Scan-order adjacent points belonging to an object form compact clusters.
  // Returns matching the known field boundary are discarded first.
  int clusterStart = -1;
  float sumX = 0.0f, sumY = 0.0f;
  int clusterSize = 0;
  float previousX = 0.0f, previousY = 0.0f;

  auto finishCluster = [&]() {
    if (clusterSize >= OPPONENT_MIN_POINTS && opponentCount < MAX_OPPONENTS) {
      const float centerX = sumX / clusterSize;
      const float centerY = sumY / clusterSize;
      float radiusSum = 0.0f;
      for (int j = clusterStart; j < clusterStart + clusterSize; ++j) {
        const ScanPoint &point = scanPoints[j];
        const float heading = point.headingDeg * LOCAL_DEG_TO_RAD;
        const float x = poseX + point.localX * cosf(heading) -
                                  point.localY * sinf(heading);
        const float y = poseY + point.localX * sinf(heading) +
                                  point.localY * cosf(heading);
        radiusSum += hypotf(x - centerX, y - centerY);
      }
      const float radius = radiusSum / clusterSize;
      if (radius >= OPPONENT_MIN_RADIUS_MM &&
          radius <= OPPONENT_MAX_RADIUS_MM &&
          insideField(centerX, centerY)) {
        const float confidence = fminf(1.0f, clusterSize / 10.0f);
        opponents[opponentCount++] = {
          true, centerX - FIELD_WIDTH_MM * 0.5f,
          centerY - FIELD_HEIGHT_MM * 0.5f, confidence, millis()
        };
      }
    }
    clusterStart = -1;
    sumX = sumY = 0.0f;
    clusterSize = 0;
  };

  for (int i = 0; i < scanPointCount; ++i) {
    const ScanPoint &point = scanPoints[i];
    const float heading = point.headingDeg * LOCAL_DEG_TO_RAD;
    const float x = poseX + point.localX * cosf(heading) -
                              point.localY * sinf(heading);
    const float y = poseY + point.localX * sinf(heading) +
                              point.localY * cosf(heading);
    const float dx = cosf(heading + atan2f(point.localY, point.localX));
    const float dy = sinf(heading + atan2f(point.localY, point.localX));
    const float expected = rayToField(poseX, poseY, dx, dy);
    const bool objectReturn = insideField(x, y) && expected > 0.0f &&
                              point.range < expected - 100.0f;
    if (!objectReturn) {
      finishCluster();
      continue;
    }

    const float gap = clusterSize == 0 ? 0.0f : hypotf(x - previousX, y - previousY);
    if (clusterSize > 0 && gap > OPPONENT_CLUSTER_GAP_MM) finishCluster();
    if (clusterSize == 0) clusterStart = i;
    sumX += x;
    sumY += y;
    previousX = x;
    previousY = y;
    ++clusterSize;
  }
  finishCluster();
  opponentTimestampMs = millis();
}

void finishScan() {
  float ranges[BIN_COUNT];
  float weights[BIN_COUNT];
  float dirX[BIN_COUNT];
  float dirY[BIN_COUNT];
  int rayCount = 0;
  collectWorldRays(ranges, weights, dirX, dirY, rayCount);
#ifdef DEBUG_LIDAR
  debugRayCount = rayCount;
#endif

  if (rayCount >= MIN_VALID_BINS) {
    Candidate best;
    if (poseInitialized) {
      const Candidate coarse = searchCandidates(
          fusedX, fusedY, 300.0f, 40.0f, ranges, weights, dirX, dirY, rayCount);
      best = searchCandidates(coarse.x, coarse.y, 60.0f, 8.0f,
                              ranges, weights, dirX, dirY, rayCount);
      // A large residual indicates a bad scan, wrong map, or heading mismatch.
      if (best.cost > 50000.0f) {
        poseInitialized = false;
      }
    }

    if (!poseInitialized) {
      const Candidate coarse = searchCandidates(
          FIELD_WIDTH_MM * 0.5f, FIELD_HEIGHT_MM * 0.5f,
          FIELD_WIDTH_MM * 0.5f, 120.0f, ranges, weights,
          dirX, dirY, rayCount);
      const Candidate medium = searchCandidates(
          coarse.x, coarse.y, 180.0f, 30.0f, ranges, weights,
          dirX, dirY, rayCount);
      best = searchCandidates(medium.x, medium.y, 40.0f, 8.0f,
                              ranges, weights, dirX, dirY, rayCount);
    }

    const unsigned long now = millis();
    const float quality = 1.0f / (1.0f + best.cost / 25000.0f);
    const bool oldPoseExpired = !robotPose.valid ||
        now - robotPose.timestampMs > POSE_TIMEOUT_MS;
    const float jump = poseInitialized
        ? hypotf(best.x - fusedX, best.y - fusedY) : 0.0f;

    if (best.cost <= 50000.0f &&
        (oldPoseExpired || jump <= 400.0f || quality >= 0.8f)) {
      if (!poseInitialized || oldPoseExpired) {
        fusedX = best.x;
        fusedY = best.y;
        poseInitialized = true;
      } else {
        const float alpha = fminf(0.75f, fmaxf(0.12f, 0.75f * quality));
        fusedX += alpha * (best.x - fusedX);
        fusedY += alpha * (best.y - fusedY);
      }
      robotPose = {true, fusedX - FIELD_WIDTH_MM * 0.5f,
                   fusedY - FIELD_HEIGHT_MM * 0.5f,
                   YAW_SIGN * currentYawDeg, quality, now};
      emitPose();
#ifdef DEBUG_LIDAR
      ++debugFixes;
#endif
      updateOpponentDetections(fusedX, fusedY);
    }
  }

  clearScan();
}

void addPacket(const LidarPacket &packet) {
  if (lastPacketStartAngle >= 0 && packet.startAngle < lastPacketStartAngle) {
    finishScan();
  }
  lastPacketStartAngle = packet.startAngle;

  const int32_t angleSpan =
      (static_cast<int32_t>(packet.endAngle) - packet.startAngle + 36000) % 36000;
  const float headingDeg = YAW_SIGN * currentYawDeg;
  for (int i = 0; i < POINTS_PER_PACKET; ++i) {
    const int32_t angleHundredths =
        (packet.startAngle + angleSpan * i / (POINTS_PER_PACKET - 1)) % 36000;
    const float localAngle = angleHundredths * 0.01f;
    if (angleBlocked(localAngle)) continue;

    const uint16_t range = packet.distanceMm[i];
    const float weight = pointWeight(range, packet.intensity[i]);
    if (weight <= 0.0f) continue;

    // LD14P angles increase clockwise in its left-handed sensor frame, while
    // field coordinates and IMU yaw are counterclockwise with +Y upward.
    // Reflect the local angle when converting it into the field frame.
    const float localRad = localAngle * LOCAL_DEG_TO_RAD;
    const float worldRad = (headingDeg - localAngle) * LOCAL_DEG_TO_RAD;
    const float worldAngle = wrap360(headingDeg - localAngle);
    int binIndex = static_cast<int>(worldAngle / (360.0f / BIN_COUNT));
    if (binIndex >= BIN_COUNT) binIndex = BIN_COUNT - 1;
    RayBin &bin = scanBins[binIndex];
    bin.weightedRange += weight * range;
    bin.weight += weight;
    bin.weightedDirX += weight * cosf(worldRad);
    bin.weightedDirY += weight * sinf(worldRad);

    if (scanPointCount < MAX_SCAN_POINTS) {
      scanPoints[scanPointCount++] = {
        range * cosf(localRad), -range * sinf(localRad),
        static_cast<float>(range), weight, headingDeg
      };
    }
  }
}

void consumeLidar() {
  while (LIDAR_UART.available() > 0) {
    const int incoming = LIDAR_UART.read();
    if (incoming < 0) break;
    const uint8_t byteValue = static_cast<uint8_t>(incoming);
#ifdef DEBUG_LIDAR
    ++debugBytes;
#endif
    switch (receiveState) {
      case WAIT_HEADER:
        if (byteValue == PACKET_HEADER) {
          packetBuffer[0] = byteValue;
          packetIndex = 1;
          receiveState = WAIT_LENGTH;
        }
        break;
      case WAIT_LENGTH:
        if (byteValue == PACKET_VERLEN) {
          packetBuffer[1] = byteValue;
          packetIndex = 2;
          receiveState = READ_PACKET;
        } else if (byteValue == PACKET_HEADER) {
          packetBuffer[0] = byteValue;
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
        packetBuffer[packetIndex++] = byteValue;
        if (packetIndex == PACKET_SIZE) {
          LidarPacket packet;
          if (parsePacket(packetBuffer, packet)) {
#ifdef DEBUG_LIDAR
            ++debugPackets;
#endif
            addPacket(packet);
          } else {
#ifdef DEBUG_LIDAR
            ++debugBadPackets;
#endif
          }
          packetIndex = 0;
          receiveState = WAIT_HEADER;
        }
        break;
    }
  }
}

void updateFieldBall() {
  const unsigned long now = millis();
  const BallPacket ball = latestBallPacket;
  const bool poseFresh = robotPose.valid &&
      now - robotPose.timestampMs <= POSE_TIMEOUT_MS;
  const bool ballFresh = ball.detected &&
      now - lastBallPacketMs <= BALL_DATA_TIMEOUT_MS;
  if (!poseFresh || !ballFresh) {
    fieldBall.valid = false;
    return;
  }

  const float angleDeg = ball.angleDeg + CAMERA_MOUNT_OFFSET_DEG;
  const float worldAngle = (robotPose.headingDeg - angleDeg) * LOCAL_DEG_TO_RAD;
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

  for (int i = 0; i < 12; ++i) {
    const int next = (i + 1) % 12;
    mapEdges[i] = {mapX[i], mapY[i], mapX[next] - mapX[i],
                   mapY[next] - mapY[i]};
  }
  clearScan();
  Serial.println("LD14P localization ready");
}

void updateLocalization() {
  consumeLidar();
  const unsigned long now = millis();
  if (robotPose.valid && now - robotPose.timestampMs > POSE_TIMEOUT_MS) {
    robotPose.valid = false;
    poseInitialized = false;
    lastPacketStartAngle = -1;
    clearScan();
    Serial.println("POSE,0");
  }
  if (robotPose.valid) {
    // Translation comes from LiDAR; heading remains live from the IMU between
    // scan matches so consumers do not use a revolution-old yaw.
    robotPose.headingDeg = YAW_SIGN * currentYawDeg;
  }
  updateFieldBall();

#ifdef DEBUG_LIDAR
  if (now - debugPrintMs >= 1000) {
    debugPrintMs = now;
    Serial.print("[LIDAR] bytes=");
    Serial.print(debugBytes);
    Serial.print(" packets=");
    Serial.print(debugPackets);
    Serial.print(" bad=");
    Serial.print(debugBadPackets);
    Serial.print(" rays=");
    Serial.print(debugRayCount);
    Serial.print(" fixes=");
    Serial.print(debugFixes);
    Serial.print(" quality=");
    Serial.print(robotPose.quality, 3);
    Serial.print(" rxBuffer=");
    Serial.println(LIDAR_RX_BUFFER_SIZE);
  }
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
