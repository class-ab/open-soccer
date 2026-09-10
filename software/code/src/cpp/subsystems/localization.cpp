#include "include/subsystems/localization.h"

#include <Arduino.h>
#include <math.h>

#include "include/subsystems/imu.h"
#include "include/subsystems/robot_config.h"
#include "include/subsystems/robot_state.h"

namespace {
constexpr uint8_t PACKET_SIZE = 47;
constexpr uint8_t PACKET_HEADER = 0x54;
constexpr uint8_t PACKET_VERLEN = 0x2C;
constexpr int POINTS_PER_PACKET = 12;
constexpr int NUM_BINS = 72;
constexpr float MAP_WIDTH_MM = 2430.0f;
constexpr float MAP_HEIGHT_MM = 1820.0f;
constexpr float NOTCH_INSET_MM = 226.0f;
constexpr float NOTCH_HEIGHT_MM = 470.0f;
constexpr float MIN_VALID_MM = 50.0f;
constexpr float MAX_VALID_MM = 12000.0f;
constexpr int MIN_RAYS_FOR_FIX = 8;
constexpr float QUALITY_SCALE = 160000.0f;
constexpr unsigned long POSE_TIMEOUT_MS = 1500;
constexpr float POLE_HALF_WIDTH_DEG = 3.0f;
const float poleCentersDeg[4] = {-67.5f, -22.5f, 22.5f, 67.5f};

// Opponent detection constants
constexpr int MAX_LIDAR_POINTS = 864;  // 72 bins * 12 points max per revolution
constexpr float OPPONENT_MIN_RADIUS_MM = 50.0f;   // 100mm diameter minimum
constexpr float OPPONENT_MAX_RADIUS_MM = 110.0f;  // 220mm diameter maximum
constexpr float CLUSTERING_DISTANCE_MM = 80.0f;   // Distance threshold for clustering
constexpr float CIRCLE_FIT_TOLERANCE_MM = 30.0f;  // Tolerance for circle fitting
constexpr int MIN_POINTS_FOR_CLUSTER = 6;         // Minimum points to form a cluster
constexpr float MIN_CLUSTER_CONFIDENCE = 0.5f;    // Minimum confidence to report

struct LidarPacket {
  uint16_t speed;
  uint16_t startAngle;
  uint16_t distanceMm[POINTS_PER_PACKET];
  uint8_t intensity[POINTS_PER_PACKET];
  uint16_t endAngle;
};

struct PoseCandidate {
  float x;
  float y;
  float cost;
};

struct LocalPoint {
  float x;
  float y;
  float intensity;
};

struct ClusterPoint {
  float x;
  float y;
  bool clustered;
};

struct DetectedRobot {
  float centerX;
  float centerY;
  float radius;
  float confidence;
};

const float mapVertices[12][2] = {
  {0.0f, MAP_HEIGHT_MM},
  {0.0f, MAP_HEIGHT_MM / 2.0f + NOTCH_HEIGHT_MM / 2.0f},
  {NOTCH_INSET_MM, MAP_HEIGHT_MM / 2.0f + NOTCH_HEIGHT_MM / 2.0f},
  {NOTCH_INSET_MM, MAP_HEIGHT_MM / 2.0f - NOTCH_HEIGHT_MM / 2.0f},
  {0.0f, MAP_HEIGHT_MM / 2.0f - NOTCH_HEIGHT_MM / 2.0f},
  {0.0f, 0.0f},
  {MAP_WIDTH_MM, 0.0f},
  {MAP_WIDTH_MM, MAP_HEIGHT_MM / 2.0f - NOTCH_HEIGHT_MM / 2.0f},
  {MAP_WIDTH_MM - NOTCH_INSET_MM, MAP_HEIGHT_MM / 2.0f - NOTCH_HEIGHT_MM / 2.0f},
  {MAP_WIDTH_MM - NOTCH_INSET_MM, MAP_HEIGHT_MM / 2.0f + NOTCH_HEIGHT_MM / 2.0f},
  {MAP_WIDTH_MM, MAP_HEIGHT_MM / 2.0f + NOTCH_HEIGHT_MM / 2.0f},
  {MAP_WIDTH_MM, MAP_HEIGHT_MM}
};

float binDistance[NUM_BINS];
float binWeight[NUM_BINS];
uint8_t rxBuffer[PACKET_SIZE];
uint8_t rxIndex = 0;
enum RxState { WAIT_HEADER, WAIT_VERLEN, READ_BODY };
RxState rxState = WAIT_HEADER;
int32_t lastStartAngle = -1;
bool poseInitialized = false;
float fusedX = MAP_WIDTH_MM / 2.0f;
float fusedY = MAP_HEIGHT_MM / 2.0f;
RobotPose robotPose = {false, 0.0f, 0.0f, 0.0f, 0.0f, 0};
FieldBall fieldBall = {false, 0.0f, 0.0f, 0.0f, 0.0f, 0};

// Opponent detection storage
LocalPoint lidarPoints[MAX_LIDAR_POINTS];
int lidarPointCount = 0;
DetectedRobot detectedOpponents[3];
int detectedOpponentCount = 0;
unsigned long lastOpponentTimestampMs = 0;

float angleDifference(float first, float second) {
  return fmodf(first - second + 540.0f, 360.0f) - 180.0f;
}

bool blockedByChassisPole(float localAngleDeg) {
  for (float poleCenter : poleCentersDeg) {
    if (fabsf(angleDifference(localAngleDeg, poleCenter)) <= POLE_HALF_WIDTH_DEG) {
      return true;
    }
  }
  return false;
}

uint16_t read16(const uint8_t *p) {
  return static_cast<uint16_t>(p[0]) |
         (static_cast<uint16_t>(p[1]) << 8);
}

uint8_t crc8(const uint8_t *data, uint8_t length) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x4D)
                         : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

bool parsePacket(const uint8_t *buffer, LidarPacket &packet) {
  if (buffer[0] != PACKET_HEADER || buffer[1] != PACKET_VERLEN ||
      crc8(buffer, PACKET_SIZE - 1) != buffer[PACKET_SIZE - 1]) {
    return false;
  }

  packet.speed = read16(buffer + 2);
  packet.startAngle = read16(buffer + 4);
  for (int i = 0; i < POINTS_PER_PACKET; i++) {
    packet.distanceMm[i] = read16(buffer + 6 + i * 3);
    packet.intensity[i] = buffer[8 + i * 3];
  }
  packet.endAngle = read16(buffer + 42);
  return true;
}

bool insideMap(float x, float y) {
  if (x < 0.0f || x > MAP_WIDTH_MM || y < 0.0f || y > MAP_HEIGHT_MM) {
    return false;
  }

  float low = MAP_HEIGHT_MM / 2.0f - NOTCH_HEIGHT_MM / 2.0f;
  float high = MAP_HEIGHT_MM / 2.0f + NOTCH_HEIGHT_MM / 2.0f;
  return !(y > low && y < high &&
           (x < NOTCH_INSET_MM || x > MAP_WIDTH_MM - NOTCH_INSET_MM));
}

float rayDistance(float rx, float ry, float dx, float dy) {
  float best = -1.0f;
  for (int i = 0; i < 12; i++) {
    int j = (i + 1) % 12;
    float ax = mapVertices[i][0];
    float ay = mapVertices[i][1];
    float ex = mapVertices[j][0] - ax;
    float ey = mapVertices[j][1] - ay;
    float denominator = dx * ey - dy * ex;
    if (fabsf(denominator) < 1e-6f) {
      continue;
    }

    float t = ((ax - rx) * ey - (ay - ry) * ex) / denominator;
    float u = ((ax - rx) * dy - (ay - ry) * dx) / denominator;
    if (t > 0.0f && u >= 0.0f && u <= 1.0f && (best < 0.0f || t < best)) {
      best = t;
    }
  }
  return best;
}

float pointWeight(uint16_t distanceMm, uint8_t intensity) {
  if (distanceMm < MIN_VALID_MM || distanceMm > MAX_VALID_MM) {
    return 0.0f;
  }
  float confidence = intensity / 255.0f;
  float midpoint = (MIN_VALID_MM + MAX_VALID_MM) * 0.35f;
  float spread = MAX_VALID_MM * 0.6f;
  float rangeWeight = 1.0f - fabsf(distanceMm - midpoint) / spread;
  return confidence * (rangeWeight < 0.05f ? 0.05f : rangeWeight);
}

// Circle fitting using least squares method
bool fitCircleToCluster(const ClusterPoint *points, int count, 
                        float &centerX, float &centerY, float &radius) {
  if (count < MIN_POINTS_FOR_CLUSTER) {
    return false;
  }

  // Calculate mean
  float meanX = 0.0f, meanY = 0.0f;
  for (int i = 0; i < count; i++) {
    if (points[i].clustered) {
      meanX += points[i].x;
      meanY += points[i].y;
    }
  }
  meanX /= count;
  meanY /= count;

  // Calculate covariance matrix elements
  float u = 0.0f, v = 0.0f, uv = 0.0f, uu = 0.0f, vv = 0.0f;
  for (int i = 0; i < count; i++) {
    if (points[i].clustered) {
      float dx = points[i].x - meanX;
      float dy = points[i].y - meanY;
      u += dx * dx;
      v += dy * dy;
      uv += dx * dy;
      uu += dx * dx * dx;
      vv += dy * dy * dy;
    }
  }
  u /= count;
  v /= count;
  uv /= count;
  uu /= count;
  vv /= count;

  // Solve for circle center
  float a = uu + uv;
  float b = uv + vv;
  // float c = 0.5f * (uu * uu + 2 * uv * uv + vv * vv - u * u - 2 * u * v + u * v - v * v);
  
  float det = a * b - uv * uv;
  if (fabsf(det) < 1e-6f) {
    return false;
  }

  float uc = (b * (uu + uv) - uv * (uv + vv)) / (2.0f * det);
  float vc = (a * (uv + vv) - uv * (uu + uv)) / (2.0f * det);

  centerX = meanX + uc;
  centerY = meanY + vc;

  // Calculate radius
  radius = 0.0f;
  for (int i = 0; i < count; i++) {
    if (points[i].clustered) {
      float dx = points[i].x - centerX;
      float dy = points[i].y - centerY;
      radius += sqrtf(dx * dx + dy * dy);
    }
  }
  radius /= count;

  return radius >= OPPONENT_MIN_RADIUS_MM && radius <= OPPONENT_MAX_RADIUS_MM;
}

// Cluster LiDAR points to find circular objects
void detectOpponentClusters() {
  if (lidarPointCount < MIN_POINTS_FOR_CLUSTER) {
    detectedOpponentCount = 0;
    return;
  }

  ClusterPoint *clusterPoints = (ClusterPoint *)malloc(lidarPointCount * sizeof(ClusterPoint));
  if (!clusterPoints) {
    detectedOpponentCount = 0;
    return;
  }

  // Initialize cluster points
  for (int i = 0; i < lidarPointCount; i++) {
    clusterPoints[i].x = lidarPoints[i].x;
    clusterPoints[i].y = lidarPoints[i].y;
    clusterPoints[i].clustered = false;
  }

  detectedOpponentCount = 0;

  // Find clusters using simple connectivity
  for (int seed = 0; seed < lidarPointCount && detectedOpponentCount < 3; seed++) {
    if (clusterPoints[seed].clustered) {
      continue;
    }

    // BFS-like clustering
    ClusterPoint cluster[MAX_LIDAR_POINTS];
    int clusterSize = 0;
    int queue[MAX_LIDAR_POINTS];
    int queueHead = 0, queueTail = 0;

    queue[queueTail++] = seed;
    clusterPoints[seed].clustered = true;

    while (queueHead < queueTail && queueTail < MAX_LIDAR_POINTS) {
      int current = queue[queueHead++];
      cluster[clusterSize].x = clusterPoints[current].x;
      cluster[clusterSize].y = clusterPoints[current].y;
      cluster[clusterSize].clustered = true;
      clusterSize++;

      // Find neighbors
      for (int j = 0; j < lidarPointCount; j++) {
        if (clusterPoints[j].clustered) {
          continue;
        }

        float dx = clusterPoints[current].x - clusterPoints[j].x;
        float dy = clusterPoints[current].y - clusterPoints[j].y;
        float distance = sqrtf(dx * dx + dy * dy);

        if (distance <= CLUSTERING_DISTANCE_MM) {
          clusterPoints[j].clustered = true;
          queue[queueTail++] = j;
        }
      }
    }

    if (clusterSize >= MIN_POINTS_FOR_CLUSTER) {
      float centerX, centerY, radius;
      if (fitCircleToCluster(cluster, clusterSize, centerX, centerY, radius)) {
        // Validate that the circle is reasonably consistent
        float totalError = 0.0f;
        for (int i = 0; i < clusterSize; i++) {
          float dx = cluster[i].x - centerX;
          float dy = cluster[i].y - centerY;
          float pointRadius = sqrtf(dx * dx + dy * dy);
          totalError += fabsf(pointRadius - radius);
        }
        float avgError = totalError / clusterSize;

        if (avgError <= CIRCLE_FIT_TOLERANCE_MM) {
          float confidence = 1.0f - (avgError / (radius + 1.0f));
          confidence = constrain(confidence, MIN_CLUSTER_CONFIDENCE, 1.0f);

          detectedOpponents[detectedOpponentCount] = {
            centerX, centerY, radius, confidence
          };
          detectedOpponentCount++;
        }
      }
    }
  }

  free(clusterPoints);
  lastOpponentTimestampMs = millis();
}

// Convert LiDAR readings to world coordinates and store them
void recordLidarPoint(float localAngleDeg, uint16_t distanceMm, uint8_t intensity) {
  if (lidarPointCount >= MAX_LIDAR_POINTS) {
    return;
  }

  float weight = pointWeight(distanceMm, intensity);
  if (weight <= 0.0f) {
    return;
  }

  // Skip if blocked by chassis pole
  if (blockedByChassisPole(localAngleDeg)) {
    return;
  }

  // Convert to local cartesian coordinates
  float localAngleRad = localAngleDeg * PI / 180.0f;
  float localX = distanceMm * cosf(localAngleRad);
  float localY = distanceMm * sinf(localAngleRad);

  // Convert to world coordinates using robot pose
  if (robotPose.valid) {
    float headingRad = robotPose.headingDeg * PI / 180.0f;
    float cosH = cosf(headingRad);
    float sinH = sinf(headingRad);

    float worldX = robotPose.xMm + localX * cosH - localY * sinH;
    float worldY = robotPose.yMm + localX * sinH + localY * cosH;

    // Only store if within map bounds
    if (insideMap(worldX, worldY)) {
      lidarPoints[lidarPointCount].x = worldX;
      lidarPoints[lidarPointCount].y = worldY;
      lidarPoints[lidarPointCount].intensity = weight;
      lidarPointCount++;
    }
  }
}

PoseCandidate searchPose(float centerX, float centerY, float halfRange,
                         float step, const float *measurements,
                         const float *weights, const float *dirX,
                         const float *dirY, int count) {
  PoseCandidate best = {centerX, centerY, 1e18f};
  for (float x = centerX - halfRange; x <= centerX + halfRange; x += step) {
    for (float y = centerY - halfRange; y <= centerY + halfRange; y += step) {
      if (!insideMap(x, y)) {
        continue;
      }

      float cost = 0.0f;
      float weightSum = 0.0f;
      for (int i = 0; i < count; i++) {
        float expected = rayDistance(x, y, dirX[i], dirY[i]);
        if (expected < 0.0f) {
          expected = MAX_VALID_MM;
        }
        float error = measurements[i] - expected;
        cost += weights[i] * error * error;
        weightSum += weights[i];
      }
      if (weightSum > 0.0f && cost / weightSum < best.cost) {
        best = {x, y, cost / weightSum};
      }
    }
  }
  return best;
}

void resetBins() {
  for (int i = 0; i < NUM_BINS; i++) {
    binDistance[i] = 0.0f;
    binWeight[i] = 0.0f;
  }
}

void finalizeRevolution() {
  float measurements[NUM_BINS];
  float weights[NUM_BINS];
  float dirX[NUM_BINS];
  float dirY[NUM_BINS];
  int count = 0;

  for (int bin = 0; bin < NUM_BINS; bin++) {
    if (binWeight[bin] <= 0.0f) {
      continue;
    }
    float localAngle = (bin + 0.5f) * (360.0f / NUM_BINS);
    float worldAngle = (localAngle + YAW_SIGN * currentYawDeg) * PI / 180.0f;
    measurements[count] = binDistance[bin] / binWeight[bin];
    weights[count] = binWeight[bin];
    dirX[count] = cosf(worldAngle);
    dirY[count] = sinf(worldAngle);
    count++;
  }

  if (count < MIN_RAYS_FOR_FIX) {
    return;
  }

  PoseCandidate coarse = searchPose(MAP_WIDTH_MM / 2.0f, MAP_HEIGHT_MM / 2.0f,
                                    MAP_WIDTH_MM / 2.0f, 100.0f, measurements,
                                    weights, dirX, dirY, count);
  PoseCandidate medium = searchPose(coarse.x, coarse.y, 150.0f, 20.0f,
                                    measurements, weights, dirX, dirY, count);
  PoseCandidate fine = searchPose(medium.x, medium.y, 25.0f, 4.0f,
                                  measurements, weights, dirX, dirY, count);
  float quality = 1.0f / (1.0f + fine.cost / QUALITY_SCALE);

  if (!poseInitialized) {
    fusedX = fine.x;
    fusedY = fine.y;
    poseInitialized = true;
  } else {
    float alpha = constrain(0.6f * quality, 0.05f, 0.9f);
    fusedX += alpha * (fine.x - fusedX);
    fusedY += alpha * (fine.y - fusedY);
  }

  robotPose = {true, fusedX - MAP_WIDTH_MM / 2.0f,
               fusedY - MAP_HEIGHT_MM / 2.0f,
               YAW_SIGN * currentYawDeg, quality, millis()};
}

void handlePacket(const LidarPacket &packet) {
  if (lastStartAngle >= 0 && packet.startAngle < lastStartAngle) {
    finalizeRevolution();
    detectOpponentClusters();  // Detect opponents after each revolution
    lidarPointCount = 0;  // Reset for next revolution
    resetBins();
  }
  lastStartAngle = packet.startAngle;

  int32_t angleStep = static_cast<int32_t>(packet.endAngle) - packet.startAngle;
  if (angleStep < 0) {
    angleStep += 36000;
  }
  for (int i = 0; i < POINTS_PER_PACKET; i++) {
    int32_t angle = (packet.startAngle + angleStep * i / 11) % 36000;
    float localAngle = angle / 100.0f;
    if (blockedByChassisPole(localAngle)) {
      continue;
    }
    float weight = pointWeight(packet.distanceMm[i], packet.intensity[i]);
    if (weight <= 0.0f) {
      continue;
    }
    // Record point for opponent detection
    recordLidarPoint(localAngle, packet.distanceMm[i], packet.intensity[i]);
    
    int bin = static_cast<int>((angle / 100.0f) / (360.0f / NUM_BINS)) % NUM_BINS;
    binDistance[bin] += weight * packet.distanceMm[i];
    binWeight[bin] += weight;
  }
}

void pollLidar() {
  while (LIDAR_UART.available()) {
    uint8_t byteValue = LIDAR_UART.read();
    if (rxState == WAIT_HEADER) {
      if (byteValue == PACKET_HEADER) {
        rxBuffer[0] = byteValue;
        rxIndex = 1;
        rxState = WAIT_VERLEN;
      }
    } else if (rxState == WAIT_VERLEN) {
      if (byteValue == PACKET_VERLEN) {
        rxBuffer[1] = byteValue;
        rxIndex = 2;
        rxState = READ_BODY;
      } else if (byteValue == PACKET_HEADER) {
        rxBuffer[0] = byteValue;
        rxIndex = 1;
      } else {
        rxState = WAIT_HEADER;
      }
    } else {
      rxBuffer[rxIndex++] = byteValue;
      if (rxIndex >= PACKET_SIZE) {
        LidarPacket packet;
        if (parsePacket(rxBuffer, packet)) {
          handlePacket(packet);
        }
        rxIndex = 0;
        rxState = WAIT_HEADER;
      }
    }
  }
}

void updateFieldBall() {
  BallPacket ball = latestBallPacket;
  unsigned long now = millis();
  bool poseFresh = robotPose.valid && now - robotPose.timestampMs <= POSE_TIMEOUT_MS;
  bool ballFresh = ball.detected && now - lastBallPacketMs <= BALL_DATA_TIMEOUT_MS;
  if (!poseFresh || !ballFresh) {
    fieldBall.valid = false;
    return;
  }

  float angleDeg = ball.angleDeg + CAMERA_MOUNT_OFFSET_DEG;
  float worldAngle = (robotPose.headingDeg - angleDeg) * PI / 180.0f;
  float distanceMm = ball.distanceCM * 10.0f;
  fieldBall = {true,
               robotPose.xMm + distanceMm * cosf(worldAngle),
               robotPose.yMm + distanceMm * sinf(worldAngle),
               ball.distanceCM, angleDeg, lastBallPacketMs};
}
}  // namespace

void initLocalization() {
  LIDAR_UART.begin(LIDAR_UART_BAUD);
  resetBins();
  Serial.println("LD14P localization UART receiver ready");
}

void updateLocalization() {
  pollLidar();
  updateFieldBall();
}

void getRobotPose(RobotPose &out) {
  out = robotPose;
}

void getFieldBall(FieldBall &out) {
  out = fieldBall;
}

void getOpponents(OpponentRobot *out, int maxOpponents, int &count) {
  count = 0;
  if (!out || maxOpponents <= 0) {
    return;
  }

  unsigned long now = millis();
  // Only provide opponents if they were detected recently
  if (now - lastOpponentTimestampMs > 500) {
    return;
  }

  for (int i = 0; i < detectedOpponentCount && count < maxOpponents; i++) {
    if (detectedOpponents[i].confidence >= MIN_CLUSTER_CONFIDENCE) {
      out[count] = {
        true,
        detectedOpponents[i].centerX,
        detectedOpponents[i].centerY,
        detectedOpponents[i].confidence,
        lastOpponentTimestampMs
      };
      count++;
    }
  }
}

// X DIRECTION IS LONG SIDE OF FIELD (i think)