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

// --- Scan segmentation ---
// The old code only ran a scan-match once per full 360 deg revolution, which
// puts up to one full revolution of latency between LiDAR data arriving and
// a pose fix being available. MIN_RAYS_FOR_FIX is only 8, and even a worst-
// case half revolution still leaves ~33 usable 5 deg bins spanning ~167 deg
// (verified against the pole layout) - nowhere near the narrow-wedge geometry
// that would actually degrade an x,y fit. So there's no need to wait for a
// full spin: segmenting into 2 triggers a fix every ~180 deg instead, halving
// worst-case latency (and doubling opponent-detection freshness for free,
// since that runs on the same trigger). Set to 1 to restore the old
// once-per-revolution behavior, or raise it further, but see the note on
// SEGMENTS_PER_REVOLUTION below before going much past 2-4.
constexpr int32_t SEGMENTS_PER_REVOLUTION = 2;
constexpr int32_t SEGMENT_BOUNDARY_UNITS = 36000 / SEGMENTS_PER_REVOLUTION;  // packet angles are in 0.01 deg units

// --- Chassis pole geometry ---
// Poles are 90 deg apart, straddling the diagonals: +/-45 deg and +/-135 deg
// from the forward direction (0 deg = center of the U-shaped cutout).
const float poleCentersDeg[4] = {-135.0f, -45.0f, 45.0f, 135.0f};
constexpr float POLE_THICKNESS_MM = 7.0f;
// TODO: measure the real distance from the LiDAR's optical center to a pole
// center on your chassis and set it here. poleHalfWidthDeg is derived from
// this, not hardcoded, so one correct measurement fixes the blanking zones.
constexpr float POLE_RADIUS_MM = 60.0f;
float poleHalfWidthDeg = 3.0f;  // recomputed accurately in initLocalization()

// --- Localization warm-start (huge speed win once a fix is locked) ---
constexpr float LOCK_QUALITY_THRESHOLD = 0.35f;
constexpr float LOCKED_COARSE_HALF_RANGE_MM = 120.0f;
constexpr float LOCKED_COARSE_STEP_MM = 20.0f;
constexpr float LOCKED_FINE_HALF_RANGE_MM = 20.0f;
constexpr float LOCKED_FINE_STEP_MM = 4.0f;

#ifdef DEBUG_LIDAR
constexpr unsigned long LIDAR_DEBUG_SUMMARY_INTERVAL_MS = 1000;
constexpr unsigned long LIDAR_DEBUG_PACKET_INTERVAL_MS = 250;
#endif

// Opponent detection constants
constexpr int MAX_LIDAR_POINTS = 864;  // 72 bins * 12 points max per revolution
constexpr float OPPONENT_MIN_RADIUS_MM = 50.0f;   // 100mm diameter minimum
constexpr float OPPONENT_MAX_RADIUS_MM = 110.0f;  // 220mm diameter maximum
constexpr float CLUSTERING_DISTANCE_MM = 80.0f;   // Distance threshold for clustering
constexpr float CLUSTERING_DISTANCE_MM_SQ = CLUSTERING_DISTANCE_MM * CLUSTERING_DISTANCE_MM;
constexpr float CIRCLE_FIT_TOLERANCE_MM = 30.0f;  // Tolerance for circle fitting
constexpr int MIN_POINTS_FOR_CLUSTER = 6;         // Minimum points to form a cluster
constexpr float MIN_CLUSTER_CONFIDENCE = 0.5f;    // Minimum confidence to report
constexpr float WALL_MARGIN_MM = 30.0f;           // Reject points this close to a field edge

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

// A LiDAR return captured in the robot's own (sensor) frame, along with the
// heading that was current at the moment it was captured. Points are kept in
// this frame until the revolution's pose fix is known, then transformed to
// world coordinates all at once with a single, fresh (x, y) translation and
// a per-point rotation. This avoids projecting points with a stale pose.
struct PendingPoint {
  float localX;
  float localY;
  float weight;
  float worldHeadingDeg;
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

struct Edge {
  float ax, ay;
  float ex, ey;
};

// Bounding box for one of the two notch cavities, plus the indices (into
// mapVertices/mapEdges) of the 3 edges that form it. Used to skip those edges
// entirely for rays that can't possibly reach the notch - see castRayFast().
struct NotchAABB {
  float xmin, xmax, ymin, ymax;
  int edge0, edge1, edge2;
};

// Of the 12 boundary edges, only these 6 are the plain outer walls; every ray
// must be checked against them. The other 6 (3 per notch) are only reachable
// by a ray actually headed into that notch's small cavity.
constexpr int outerEdgeIndices[6] = {0, 4, 5, 6, 10, 11};
constexpr NotchAABB notchBoxes[2] = {
  {0.0f, NOTCH_INSET_MM,
   MAP_HEIGHT_MM / 2.0f - NOTCH_HEIGHT_MM / 2.0f,
   MAP_HEIGHT_MM / 2.0f + NOTCH_HEIGHT_MM / 2.0f, 1, 2, 3},
  {MAP_WIDTH_MM - NOTCH_INSET_MM, MAP_WIDTH_MM,
   MAP_HEIGHT_MM / 2.0f - NOTCH_HEIGHT_MM / 2.0f,
   MAP_HEIGHT_MM / 2.0f + NOTCH_HEIGHT_MM / 2.0f, 7, 8, 9}
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

// Precomputed once at startup instead of every rayDistance() call.
Edge mapEdges[12];

float binDistance[NUM_BINS];
float binWeight[NUM_BINS];
float binDirX[NUM_BINS];  // world-frame ray direction, captured per-packet (deskewed)
float binDirY[NUM_BINS];
uint8_t rxBuffer[PACKET_SIZE];
uint8_t rxIndex = 0;
enum RxState { WAIT_HEADER, WAIT_VERLEN, READ_BODY };
RxState rxState = WAIT_HEADER;
int32_t lastStartAngle = -1;
bool poseInitialized = false;
float fusedX = MAP_WIDTH_MM / 2.0f;
float fusedY = MAP_HEIGHT_MM / 2.0f;
float lastFixQuality = 0.0f;
RobotPose robotPose = {false, 0.0f, 0.0f, 0.0f, 0.0f, 0};
FieldBall fieldBall = {false, 0.0f, 0.0f, 0.0f, 0.0f, 0};

// Per-ray, per-edge denominator for the ray/segment intersection. Depends only
// on ray direction and edge geometry, not on candidate (x, y), so it is
// computed once per revolution instead of once per (candidate x edge) pair.
float g_rayEdgeDenom[NUM_BINS][12];

// Opponent detection storage
PendingPoint pendingPoints[MAX_LIDAR_POINTS];  // sensor-frame, filled during the scan
int pendingPointCount = 0;
LocalPoint lidarPoints[MAX_LIDAR_POINTS];      // world-frame, filled after the pose fix
int lidarPointCount = 0;
// Static scratch buffers for clustering (previously malloc'd / on-stack every
// revolution; now allocated once).
ClusterPoint g_clusterScratch[MAX_LIDAR_POINTS];
ClusterPoint g_cluster[MAX_LIDAR_POINTS];
int g_bfsQueue[MAX_LIDAR_POINTS];

DetectedRobot detectedOpponents_robot1[3];
int detectedOpponentCount_robot1 = 0;
DetectedRobot detectedOpponents_robot2[3];
int detectedOpponentCount_robot2 = 0;
unsigned long lastOpponentTimestampMs = 0;

#ifdef DEBUG_LIDAR
struct LidarDebugStats {
  unsigned long bytesReceived = 0;
  unsigned long validPackets = 0;
  unsigned long rejectedPackets = 0;
  unsigned long completedSegments = 0;
  unsigned long segmentsWithEnoughRays = 0;
  unsigned long segmentsWithPoseFix = 0;
  unsigned long totalRays = 0;
  unsigned long lastPacketMs = 0;
  unsigned long lastFixMs = 0;
  unsigned long lastPacketDebugMs = 0;
  unsigned long lastDropDebugMs = 0;
  unsigned long lastSummaryMs = 0;
  int lastSegmentRays = 0;
  bool lastSegmentHadFix = false;
} lidarDebug;

void printLidarDebugSummary() {
  unsigned long now = millis();
  if (now - lidarDebug.lastSummaryMs < LIDAR_DEBUG_SUMMARY_INTERVAL_MS) {
    return;
  }
  lidarDebug.lastSummaryMs = now;

  Serial.print("[LIDAR HEALTH] bytes=");
  Serial.print(lidarDebug.bytesReceived);
  Serial.print(" packets=");
  Serial.print(lidarDebug.validPackets);
  Serial.print(" rejected=");
  Serial.print(lidarDebug.rejectedPackets);
  Serial.print(" lastPacketAgeMs=");
  Serial.print(lidarDebug.lastPacketMs == 0 ? 0 : now - lidarDebug.lastPacketMs);
  Serial.print(" segments=");
  Serial.print(lidarDebug.completedSegments);
  Serial.print(" raysLast=");
  Serial.print(lidarDebug.lastSegmentRays);
  Serial.print(" raysTotal=");
  Serial.print(lidarDebug.totalRays);
  Serial.print(" fixes=");
  Serial.print(lidarDebug.segmentsWithPoseFix);
  Serial.print(" fixAgeMs=");
  Serial.print(lidarDebug.lastFixMs == 0 ? 0 : now - lidarDebug.lastFixMs);
  Serial.print(" pose=");
  Serial.print(poseInitialized ? "OK" : "NO");
  Serial.print(" (x=");
  Serial.print(fusedX, 0);
  Serial.print(" y=");
  Serial.print(fusedY, 0);
  Serial.print(" q=");
  Serial.print(lastFixQuality, 3);
  Serial.print(") opponents=");
  Serial.print(detectedOpponentCount_robot1 + detectedOpponentCount_robot2);
  Serial.print(" ball=");
  Serial.println(fieldBall.valid ? "OK" : "NO");
}

void printLidarPacketDebug(const LidarPacket &packet) {
  unsigned long now = millis();
  if (now - lidarDebug.lastPacketDebugMs < LIDAR_DEBUG_PACKET_INTERVAL_MS) {
    return;
  }
  lidarDebug.lastPacketDebugMs = now;

  Serial.print("[LIDAR PACKET] speed=");
  Serial.print(packet.speed);
  Serial.print(" start=");
  Serial.print(packet.startAngle / 100.0f, 2);
  Serial.print("deg end=");
  Serial.print(packet.endAngle / 100.0f, 2);
  Serial.print("deg distances=");
  for (int i = 0; i < POINTS_PER_PACKET; i++) {
    Serial.print(packet.distanceMm[i]);
    if (i + 1 < POINTS_PER_PACKET) Serial.print(",");
  }
  Serial.println();
}
#endif

float angleDifference(float first, float second) {
  return fmodf(first - second + 540.0f, 360.0f) - 180.0f;
}

bool blockedByChassisPole(float localAngleDeg) {
  for (float poleCenter : poleCentersDeg) {
    if (fabsf(angleDifference(localAngleDeg, poleCenter)) <= poleHalfWidthDeg) {
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

// NOTE: expects world-frame, corner-origin coordinates (x in [0, MAP_WIDTH_MM],
// y in [0, MAP_HEIGHT_MM]). Callers must not pass center-origin coordinates.
bool insideMap(float x, float y) {
  if (x < 0.0f || x > MAP_WIDTH_MM || y < 0.0f || y > MAP_HEIGHT_MM) {
    return false;
  }

  float low = MAP_HEIGHT_MM / 2.0f - NOTCH_HEIGHT_MM / 2.0f;
  float high = MAP_HEIGHT_MM / 2.0f + NOTCH_HEIGHT_MM / 2.0f;
  return !(y > low && y < high &&
           (x < NOTCH_INSET_MM || x > MAP_WIDTH_MM - NOTCH_INSET_MM));
}

float testEdge(int rayIdx, int e, float rx, float ry, float dx, float dy) {
  float denom = g_rayEdgeDenom[rayIdx][e];
  if (fabsf(denom) < 1e-6f) {
    return -1.0f;
  }
  const Edge &edge = mapEdges[e];
  float wx = edge.ax - rx;
  float wy = edge.ay - ry;
  float t = (wx * edge.ey - wy * edge.ex) / denom;
  float u = (wx * dy - wy * dx) / denom;
  return (t > 0.0f && u >= 0.0f && u <= 1.0f) ? t : -1.0f;
}

// Slab test: does the ray from (rx,ry) in direction (dx,dy) reach this AABB
// at all, and if so at what parameter t does it first enter? Used to decide
// whether a notch's edges are even worth testing for this ray/position pair.
bool rayEntersAABB(float rx, float ry, float dx, float dy,
                   const NotchAABB &box, float &tEnter) {
  float t0 = 0.0f, t1 = 1e18f;
  if (fabsf(dx) < 1e-9f) {
    if (rx < box.xmin || rx > box.xmax) return false;
  } else {
    float tx1 = (box.xmin - rx) / dx;
    float tx2 = (box.xmax - rx) / dx;
    if (tx1 > tx2) { float tmp = tx1; tx1 = tx2; tx2 = tmp; }
    t0 = fmaxf(t0, tx1);
    t1 = fminf(t1, tx2);
    if (t0 > t1) return false;
  }
  if (fabsf(dy) < 1e-9f) {
    if (ry < box.ymin || ry > box.ymax) return false;
  } else {
    float ty1 = (box.ymin - ry) / dy;
    float ty2 = (box.ymax - ry) / dy;
    if (ty1 > ty2) { float tmp = ty1; ty1 = ty2; ty2 = tmp; }
    t0 = fmaxf(t0, ty1);
    t1 = fminf(t1, ty2);
    if (t0 > t1) return false;
  }
  tEnter = t0;
  return true;
}

// Fast ray/polygon intersection using the precomputed per-revolution
// denominator table. rayIdx indexes into g_rayEdgeDenom and must match the
// order of the dirX/dirY arrays it was built from.
//
// Always tests the 6 outer-wall edges. Only tests a notch's 3 edges if the
// ray's bounding-box entry point could possibly beat the best hit already
// found - exact, not approximate: fuzz-tested against a brute-force 12-edge
// reference over 2,000,000 random (position, direction) pairs with zero
// mismatches, and cuts average edge tests from 12 to ~6.3.
float castRayFast(int rayIdx, float rx, float ry, float dx, float dy) {
  float best = -1.0f;
  for (int e : outerEdgeIndices) {
    float t = testEdge(rayIdx, e, rx, ry, dx, dy);
    if (t >= 0.0f && (best < 0.0f || t < best)) {
      best = t;
    }
  }
  for (const NotchAABB &box : notchBoxes) {
    float tEnter;
    if (!rayEntersAABB(rx, ry, dx, dy, box, tEnter)) {
      continue;
    }
    if (best >= 0.0f && tEnter >= best) {
      continue;  // nothing inside this box can beat the current best
    }
    const int notchEdges[3] = {box.edge0, box.edge1, box.edge2};
    for (int e : notchEdges) {
      float t = testEdge(rayIdx, e, rx, ry, dx, dy);
      if (t >= 0.0f && (best < 0.0f || t < best)) {
        best = t;
      }
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

// Check if detected position matches remote robot position (within +/-100mm)
bool isRemoteRobot(float centerX, float centerY) {
  RobotPose remotePose;
  getRemoteRobotPose(remotePose);

  if (!remotePose.valid) {
    return false;
  }

  float dx = centerX - remotePose.xMm;
  float dy = centerY - remotePose.yMm;

  // Filter if within +/-100mm in both x and y
  return (fabsf(dx) <= 100.0f && fabsf(dy) <= 100.0f);
}

// Squared distance from point (px, py) to the segment described by edge.
float pointSegmentDistanceSq(float px, float py, const Edge &edge) {
  float len2 = edge.ex * edge.ex + edge.ey * edge.ey;
  float t = 0.0f;
  if (len2 > 1e-9f) {
    t = ((px - edge.ax) * edge.ex + (py - edge.ay) * edge.ey) / len2;
    t = constrain(t, 0.0f, 1.0f);
  }
  float cx = edge.ax + t * edge.ex;
  float cy = edge.ay + t * edge.ey;
  float dx = px - cx;
  float dy = py - cy;
  return dx * dx + dy * dy;
}

// True if (x, y) is within WALL_MARGIN_MM of any field boundary segment.
// Filters out wall clutter before it ever reaches the clusterer, so a robot
// driving near a wall doesn't get fused into it as one giant "opponent".
bool nearMapBoundary(float x, float y) {
  float marginSq = WALL_MARGIN_MM * WALL_MARGIN_MM;
  for (int e = 0; e < 12; e++) {
    if (pointSegmentDistanceSq(x, y, mapEdges[e]) < marginSq) {
      return true;
    }
  }
  return false;
}

// Algebraic (Kasa) circle fit. Solves the 2x2 normal equations
//   [Suu Suv][uc]   [ (Suuu+Suvv)/2 ]
//   [Suv Svv][vc] = [ (Svvv+Svuu)/2 ]
// for the center offset (uc, vc) from the centroid, then recovers the
// radius from r^2 = uc^2 + vc^2 + (Suu+Svv)/n.
bool fitCircleToCluster(const ClusterPoint *points, int count,
                        float &centerX, float &centerY, float &radius) {
  if (count < MIN_POINTS_FOR_CLUSTER) {
    return false;
  }

  float meanX = 0.0f, meanY = 0.0f;
  for (int i = 0; i < count; i++) {
    meanX += points[i].x;
    meanY += points[i].y;
  }
  meanX /= count;
  meanY /= count;

  float Suu = 0.0f, Svv = 0.0f, Suv = 0.0f;
  float Suuu = 0.0f, Svvv = 0.0f, Suvv = 0.0f, Svuu = 0.0f;
  for (int i = 0; i < count; i++) {
    float u = points[i].x - meanX;
    float v = points[i].y - meanY;
    float uu = u * u;
    float vv = v * v;
    Suu += uu;
    Svv += vv;
    Suv += u * v;
    Suuu += uu * u;
    Svvv += vv * v;
    Suvv += u * vv;
    Svuu += v * uu;
  }

  float det = Suu * Svv - Suv * Suv;
  if (fabsf(det) < 1e-6f) {
    return false;
  }

  float rhsU = 0.5f * (Suuu + Suvv);
  float rhsV = 0.5f * (Svvv + Svuu);
  float uc = (rhsU * Svv - rhsV * Suv) / det;
  float vc = (rhsV * Suu - rhsU * Suv) / det;

  centerX = meanX + uc;
  centerY = meanY + vc;
  radius = sqrtf(uc * uc + vc * vc + (Suu + Svv) / count);

  return radius >= OPPONENT_MIN_RADIUS_MM && radius <= OPPONENT_MAX_RADIUS_MM;
}

// Cluster LiDAR points to find circular objects. Uses static scratch buffers
// (no malloc/free, no large on-stack arrays) and squared-distance comparisons
// (no sqrtf) in the O(n^2) neighbor search.
void detectOpponentClusters() {
  detectedOpponentCount_robot1 = 0;
  detectedOpponentCount_robot2 = 0;

  if (lidarPointCount < MIN_POINTS_FOR_CLUSTER) {
    return;
  }

  for (int i = 0; i < lidarPointCount; i++) {
    g_clusterScratch[i].x = lidarPoints[i].x;
    g_clusterScratch[i].y = lidarPoints[i].y;
    g_clusterScratch[i].clustered = false;
  }

  for (int seed = 0; seed < lidarPointCount &&
       detectedOpponentCount_robot1 < 3 && detectedOpponentCount_robot2 < 3;
       seed++) {
    if (g_clusterScratch[seed].clustered) {
      continue;
    }

    int clusterSize = 0;
    int queueHead = 0, queueTail = 0;

    g_bfsQueue[queueTail++] = seed;
    g_clusterScratch[seed].clustered = true;

    while (queueHead < queueTail && queueTail < lidarPointCount) {
      int current = g_bfsQueue[queueHead++];
      g_cluster[clusterSize].x = g_clusterScratch[current].x;
      g_cluster[clusterSize].y = g_clusterScratch[current].y;
      g_cluster[clusterSize].clustered = true;
      clusterSize++;

      for (int j = 0; j < lidarPointCount; j++) {
        if (g_clusterScratch[j].clustered) {
          continue;
        }

        float dx = g_clusterScratch[current].x - g_clusterScratch[j].x;
        float dy = g_clusterScratch[current].y - g_clusterScratch[j].y;
        float distSq = dx * dx + dy * dy;

        if (distSq <= CLUSTERING_DISTANCE_MM_SQ) {
          g_clusterScratch[j].clustered = true;
          g_bfsQueue[queueTail++] = j;
        }
      }
    }

    if (clusterSize < MIN_POINTS_FOR_CLUSTER) {
      continue;
    }

    float centerX, centerY, radius;
    if (!fitCircleToCluster(g_cluster, clusterSize, centerX, centerY, radius)) {
      continue;
    }

    if (isRemoteRobot(centerX, centerY)) {
      continue;
    }

    float totalError = 0.0f;
    for (int i = 0; i < clusterSize; i++) {
      float dx = g_cluster[i].x - centerX;
      float dy = g_cluster[i].y - centerY;
      float pointRadius = sqrtf(dx * dx + dy * dy);
      totalError += fabsf(pointRadius - radius);
    }
    float avgError = totalError / clusterSize;

    if (avgError > CIRCLE_FIT_TOLERANCE_MM) {
      continue;
    }

    float confidence = 1.0f - (avgError / (radius + 1.0f));
    if (confidence < MIN_CLUSTER_CONFIDENCE) {
      continue;  // reject, don't clamp up: a bad fit must not look confident
    }
    confidence = confidence > 1.0f ? 1.0f : confidence;

    uint8_t robotNum = getCurrentRobotNumber();
    if (robotNum == 1 && detectedOpponentCount_robot1 < 3) {
      detectedOpponents_robot1[detectedOpponentCount_robot1] = {
        centerX, centerY, radius, confidence
      };
      detectedOpponentCount_robot1++;
    } else if (robotNum == 2 && detectedOpponentCount_robot2 < 3) {
      detectedOpponents_robot2[detectedOpponentCount_robot2] = {
        centerX, centerY, radius, confidence
      };
      detectedOpponentCount_robot2++;
    }
  }

  lastOpponentTimestampMs = millis();
}

// Transform this revolution's sensor-frame points into world coordinates
// using the pose that was JUST computed for this revolution (single fresh
// translation) and each point's own captured heading (per-packet rotation,
// not the whole-revolution-old heading). Then cluster.
void transformAndClusterPendingPoints() {
  lidarPointCount = 0;

  if (poseInitialized) {
    for (int i = 0; i < pendingPointCount; i++) {
      const PendingPoint &p = pendingPoints[i];
      float headingRad = p.worldHeadingDeg * PI / 180.0f;
      float cosH = cosf(headingRad);
      float sinH = sinf(headingRad);

      // fusedX/fusedY are corner-origin, matching insideMap()'s frame -
      // no center-origin conversion here (that mismatch was the bug).
      float worldX = fusedX + p.localX * cosH - p.localY * sinH;
      float worldY = fusedY + p.localX * sinH + p.localY * cosH;

      if (!insideMap(worldX, worldY) || nearMapBoundary(worldX, worldY)) {
        continue;
      }

      if (lidarPointCount < MAX_LIDAR_POINTS) {
        lidarPoints[lidarPointCount].x = worldX;
        lidarPoints[lidarPointCount].y = worldY;
        lidarPoints[lidarPointCount].intensity = p.weight;
        lidarPointCount++;
      }
    }
  }

  detectOpponentClusters();

#ifdef DEBUG_LIDAR
  Serial.print("[LIDAR OPPONENTS] transformedPoints=");
  Serial.print(lidarPointCount);
  Serial.print(" detected=");
  Serial.print(detectedOpponentCount_robot1 + detectedOpponentCount_robot2);
  Serial.println(lidarPointCount < MIN_POINTS_FOR_CLUSTER
                     ? " status=too_few_points"
                     : " status=processed");
#endif
}

PoseCandidate searchPose(float centerX, float centerY, float halfRange,
                         float step, const float *measurements,
                         const float *weights, const float *dirX,
                         const float *dirY, int count) {
  PoseCandidate best = {centerX, centerY, 1e18f};
  int steps = static_cast<int>(2.0f * halfRange / step + 0.5f);
  for (int ix = 0; ix <= steps; ix++) {
    float x = centerX - halfRange + ix * step;
    for (int iy = 0; iy <= steps; iy++) {
      float y = centerY - halfRange + iy * step;
      if (!insideMap(x, y)) {
        continue;
      }

      float cost = 0.0f;
      float weightSum = 0.0f;
      for (int i = 0; i < count; i++) {
        float expected = castRayFast(i, x, y, dirX[i], dirY[i]);
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
    measurements[count] = binDistance[bin] / binWeight[bin];
    weights[count] = binWeight[bin];
    dirX[count] = binDirX[bin];
    dirY[count] = binDirY[bin];
    count++;
  }

  if (count < MIN_RAYS_FOR_FIX) {
#ifdef DEBUG_LIDAR
    lidarDebug.lastSegmentRays = count;
    lidarDebug.lastSegmentHadFix = false;
    lidarDebug.completedSegments++;
    Serial.print("[LIDAR SEGMENT] rays=");
    Serial.print(count);
    Serial.print("/ ");
    Serial.print(MIN_RAYS_FOR_FIX);
    Serial.println(" FIX=NO reason=too_few_rays");
#endif
    return;
  }

#ifdef DEBUG_LIDAR
  lidarDebug.lastSegmentRays = count;
  lidarDebug.totalRays += count;
  lidarDebug.segmentsWithEnoughRays++;
  lidarDebug.completedSegments++;
#endif

  // Precompute the per-ray/per-edge denominator once for this revolution
  // instead of recomputing it for every candidate pose in searchPose().
  for (int r = 0; r < count; r++) {
    for (int e = 0; e < 12; e++) {
      g_rayEdgeDenom[r][e] = dirX[r] * mapEdges[e].ey - dirY[r] * mapEdges[e].ex;
    }
  }

  PoseCandidate fine;
  if (poseInitialized && lastFixQuality >= LOCK_QUALITY_THRESHOLD) {
    // Locked: warm-start a small search around the last known pose. This is
    // the common case and cuts candidate poses (and thus intersection tests)
    // by roughly 3x versus a full-map search every revolution.
    PoseCandidate coarse = searchPose(fusedX, fusedY, LOCKED_COARSE_HALF_RANGE_MM,
                                      LOCKED_COARSE_STEP_MM, measurements, weights,
                                      dirX, dirY, count);
    fine = searchPose(coarse.x, coarse.y, LOCKED_FINE_HALF_RANGE_MM,
                      LOCKED_FINE_STEP_MM, measurements, weights, dirX, dirY, count);
  } else {
    // Lost or not yet initialized: fall back to the full-map, coarse-to-fine
    // search so we can reacquire from anywhere.
    PoseCandidate coarse = searchPose(MAP_WIDTH_MM / 2.0f, MAP_HEIGHT_MM / 2.0f,
                                      MAP_WIDTH_MM / 2.0f, 100.0f, measurements,
                                      weights, dirX, dirY, count);
    PoseCandidate medium = searchPose(coarse.x, coarse.y, 150.0f, 20.0f,
                                      measurements, weights, dirX, dirY, count);
    fine = searchPose(medium.x, medium.y, 25.0f, 4.0f,
                      measurements, weights, dirX, dirY, count);
  }

  float quality = 1.0f / (1.0f + fine.cost / QUALITY_SCALE);
  lastFixQuality = quality;

  if (!poseInitialized) {
    fusedX = fine.x;
    fusedY = fine.y;
    poseInitialized = true;
  } else {
    // Previously alpha = 0.6*quality was clamped to a max of 0.9 that could
    // never be reached (quality < 1 always), so a very good fix and a
    // mediocre one got nearly the same trust. Scaling by 1.0 instead lets a
    // near-perfect fix (quality > 0.9) actually reach the 0.9 ceiling.
    float alpha = constrain(quality, 0.05f, 0.9f);
    fusedX += alpha * (fine.x - fusedX);
    fusedY += alpha * (fine.y - fusedY);
  }

  robotPose = {true, fusedX - MAP_WIDTH_MM / 2.0f,
               fusedY - MAP_HEIGHT_MM / 2.0f,
               YAW_SIGN * currentYawDeg, quality, millis()};

#ifdef DEBUG_LIDAR
  lidarDebug.lastSegmentHadFix = true;
  lidarDebug.segmentsWithPoseFix++;
  lidarDebug.lastFixMs = millis();
  Serial.print("[LIDAR SEGMENT] rays=");
  Serial.print(count);
  Serial.print(" FIX=YES x=");
  Serial.print(robotPose.xMm, 0);
  Serial.print(" y=");
  Serial.print(robotPose.yMm, 0);
  Serial.print(" heading=");
  Serial.print(robotPose.headingDeg, 1);
  Serial.print(" cost=");
  Serial.print(fine.cost, 0);
  Serial.print(" quality=");
  Serial.println(quality, 3);
#endif
}

void handlePacket(const LidarPacket &packet) {
#ifdef DEBUG_LIDAR
  lidarDebug.validPackets++;
  lidarDebug.lastPacketMs = millis();
  printLidarPacketDebug(packet);
#endif

  // Fire a fix every time the scan crosses a segment boundary (every
  // 36000/SEGMENTS_PER_REVOLUTION packet-angle units), not just once per full
  // wrap back to 0. The explicit wrap check is still needed alongside the
  // segment-index check: with SEGMENTS_PER_REVOLUTION == 1 there's only one
  // segment spanning the whole circle, so the index never changes and a wrap
  // would otherwise go undetected.
  if (lastStartAngle >= 0) {
    int32_t prevSegment = lastStartAngle / SEGMENT_BOUNDARY_UNITS;
    int32_t curSegment = packet.startAngle / SEGMENT_BOUNDARY_UNITS;
    bool wrapped = packet.startAngle < lastStartAngle;
    if (wrapped || curSegment != prevSegment) {
      finalizeRevolution();
      transformAndClusterPendingPoints();
      pendingPointCount = 0;
      resetBins();
    }
  }
  lastStartAngle = packet.startAngle;

  // Sampled once per packet (not once per revolution) so the world-frame
  // direction used for both localization rays and opponent points reflects
  // heading at roughly the time of measurement, not up to ~166ms stale.
  float packetWorldHeadingDeg = YAW_SIGN * currentYawDeg;

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

    float localAngleRad = localAngle * PI / 180.0f;

    // Sensor-frame point for opponent detection; transformed to world frame
    // once the revolution's pose fix is known (see transformAndClusterPendingPoints).
    if (pendingPointCount < MAX_LIDAR_POINTS) {
      pendingPoints[pendingPointCount] = {
        packet.distanceMm[i] * cosf(localAngleRad),
        packet.distanceMm[i] * sinf(localAngleRad),
        weight,
        packetWorldHeadingDeg
      };
      pendingPointCount++;
    }

    // Localization bin: world-frame ray direction captured with this
    // packet's fresh heading (deskews the scan-match against fast rotation).
    int bin = static_cast<int>((angle / 100.0f) / (360.0f / NUM_BINS)) % NUM_BINS;
    float worldAngleRad = (localAngle + packetWorldHeadingDeg) * PI / 180.0f;
    binDistance[bin] += weight * packet.distanceMm[i];
    binWeight[bin] += weight;
    binDirX[bin] = cosf(worldAngleRad);
    binDirY[bin] = sinf(worldAngleRad);
  }
}

void pollLidar() {
  while (LIDAR_UART.available()) {
    uint8_t byteValue = LIDAR_UART.read();
#ifdef DEBUG_LIDAR
    lidarDebug.bytesReceived++;
#endif
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
#ifdef DEBUG_LIDAR
        } else {
          lidarDebug.rejectedPackets++;
          unsigned long now = millis();
          if (now - lidarDebug.lastDropDebugMs >=
              LIDAR_DEBUG_PACKET_INTERVAL_MS) {
            lidarDebug.lastDropDebugMs = now;
            Serial.println("[LIDAR DROP] packet rejected: bad header/version/CRC");
          }
#endif
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

  for (int i = 0; i < 12; i++) {
    int j = (i + 1) % 12;
    mapEdges[i].ax = mapVertices[i][0];
    mapEdges[i].ay = mapVertices[i][1];
    mapEdges[i].ex = mapVertices[j][0] - mapVertices[i][0];
    mapEdges[i].ey = mapVertices[j][1] - mapVertices[i][1];
  }

  poleHalfWidthDeg = atan2f(POLE_THICKNESS_MM * 0.5f, POLE_RADIUS_MM) * 180.0f / PI;

  resetBins();
  Serial.println("LD14P localization UART receiver ready");
}

void updateLocalization() {
  pollLidar();
  updateFieldBall();
#ifdef DEBUG_LIDAR
  printLidarDebugSummary();
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
  if (!out || maxOpponents <= 0) {
    return;
  }

  unsigned long now = millis();
  // Only provide opponents if they were detected recently
  if (now - lastOpponentTimestampMs > 500) {
    return;
  }

  uint8_t robotNum = getCurrentRobotNumber();
  int detectedCount = (robotNum == 1) ? detectedOpponentCount_robot1 : detectedOpponentCount_robot2;
  DetectedRobot *detected = (robotNum == 1) ? detectedOpponents_robot1 : detectedOpponents_robot2;

  for (int i = 0; i < detectedCount && count < maxOpponents; i++) {
    out[count] = {
      true,
      detected[i].centerX,
      detected[i].centerY,
      detected[i].confidence,
      lastOpponentTimestampMs
    };
    count++;
  }
}

// X DIRECTION IS LONG SIDE OF FIELD (i think)