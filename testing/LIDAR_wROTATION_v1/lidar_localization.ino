/* =====================================================================================
   Teensy 4.1 + LD14P LiDAR — ray-cast localization AND rotation estimation.

   IMU is dead, so heading is no longer an external input: it's solved for the same
   way position is, by grid-searching over candidate headings and scoring each one
   against the known map polygon, then temporally fusing the result.

   CAVEAT — READ THIS: the map (rectangle + two identical notches, one per short
   wall, both centered on the vertical midline) has 180-degree point symmetry about
   its center. That means a pose (x, y, theta) and its 180-degree-rotated twin
   (W-x, H-y, theta+180) produce IDENTICAL scans. This code cannot break that tie on
   its own. Options: seed fusedTheta below with a known start orientation, add some
   asymmetric feature to the room, or accept a possible 180-degree flip and correct
   for it externally (e.g. compare against a known start pose once).

   Everything else (packet parsing, PWM speed trim, pole exclusion, map geometry) is
   unchanged from the previous version.
   ===================================================================================== */

#include <math.h>
#include "lidar_types.h"

// ---- Serial ----
#define LIDAR_SERIAL      Serial3
#define LIDAR_BAUD        230400
#define DEBUG_BAUD        115200

// ---- LD14P speed control ----
#define LIDAR_PWM_PIN      14
#define LIDAR_PWM_FREQ_HZ  1000
#define TARGET_SCAN_HZ      8.0f
#define PWM_TRIM_PERIOD    20
uint8_t pwmDuty = 220;

// ---- Map geometry (mm), origin = bottom-left outer corner ----
#define MAP_WIDTH_MM       2430.0f
#define MAP_HEIGHT_MM      1820.0f
#define NOTCH_INSET_MM      226.0f
#define NOTCH_HEIGHT_MM     470.0f

// ---- Point validity ----
#define MIN_VALID_MM        50.0f
#define MAX_VALID_MM     12000.0f
#define MIN_RAYS_FOR_FIX      8

// ---- Scan binning ----
#define NUM_BINS              72

// ---- Position grid search (coarse -> fine), used once theta is known ----
#define STAGE2_HALFRANGE_MM  150.0f
#define STAGE2_STEP_MM        20.0f
#define STAGE3_HALFRANGE_MM   25.0f
#define STAGE3_STEP_MM         4.0f
#define QUALITY_SCALE      160000.0f

// ---- Rotation search ----
#define THETA_BOOT_STEP_DEG      8.0f   // full 0-360 sweep, first fix only
#define THETA_BOOT_POS_STEP_MM 150.0f   // coarse position step paired with each theta candidate
#define THETA_REFINE1_HALF_DEG   8.0f
#define THETA_REFINE1_STEP_DEG   0.5f
#define THETA_REFINE2_HALF_DEG   1.5f
#define THETA_REFINE2_STEP_DEG   0.1f

// ---- Temporal fusion ----
#define FUSION_BASE_ALPHA      0.6f
#define FUSION_MIN_ALPHA       0.05f
#define FUSION_MAX_ALPHA       0.9f

// ---- Chassis-mounted poles blocking fixed LOCAL angles (unrelated to heading) ----
#define POLE_COUNT           4
#define ROTOR_RADIUS_MM      30.0f
#define POLE_STANDOFF_MM     10.0f
#define POLE_WIDTH_MM          7.0f
#define POLE_DISTANCE_MM     (ROTOR_RADIUS_MM + POLE_STANDOFF_MM)

const float poleCenterAngleDeg[POLE_COUNT] = { -67.5f, -22.5f, 22.5f, 67.5f };
const float poleHalfWidthDeg = degrees(atanf((POLE_WIDTH_MM / 2.0f) / POLE_DISTANCE_MM));

float angleDiffDeg(float a, float b) {
  return fmodf(a - b + 540.0f, 360.0f) - 180.0f;
}

bool angleBlockedByPole(float localAngleDeg) {
  for (int i = 0; i < POLE_COUNT; i++) {
    if (fabsf(angleDiffDeg(localAngleDeg, poleCenterAngleDeg[i])) <= poleHalfWidthDeg) return true;
  }
  return false;
}

// =====================================================================================
// Map polygon
// =====================================================================================

const float mapVertices[12][2] = {
  { 0.0f,                                MAP_HEIGHT_MM },
  { 0.0f,                                MAP_HEIGHT_MM/2.0f + NOTCH_HEIGHT_MM/2.0f },
  { NOTCH_INSET_MM,                      MAP_HEIGHT_MM/2.0f + NOTCH_HEIGHT_MM/2.0f },
  { NOTCH_INSET_MM,                      MAP_HEIGHT_MM/2.0f - NOTCH_HEIGHT_MM/2.0f },
  { 0.0f,                                MAP_HEIGHT_MM/2.0f - NOTCH_HEIGHT_MM/2.0f },
  { 0.0f,                                0.0f },
  { MAP_WIDTH_MM,                        0.0f },
  { MAP_WIDTH_MM,                        MAP_HEIGHT_MM/2.0f - NOTCH_HEIGHT_MM/2.0f },
  { MAP_WIDTH_MM - NOTCH_INSET_MM,       MAP_HEIGHT_MM/2.0f - NOTCH_HEIGHT_MM/2.0f },
  { MAP_WIDTH_MM - NOTCH_INSET_MM,       MAP_HEIGHT_MM/2.0f + NOTCH_HEIGHT_MM/2.0f },
  { MAP_WIDTH_MM,                        MAP_HEIGHT_MM/2.0f + NOTCH_HEIGHT_MM/2.0f },
  { MAP_WIDTH_MM,                        MAP_HEIGHT_MM }
};
#define NUM_EDGES 12

float rayDistanceToPolygon(float rx, float ry, float dx, float dy) {
  float best = -1.0f;
  for (int i = 0; i < NUM_EDGES; i++) {
    float ax = mapVertices[i][0];
    float ay = mapVertices[i][1];
    int j = (i + 1) % NUM_EDGES;
    float ex = mapVertices[j][0] - ax;
    float ey = mapVertices[j][1] - ay;

    float denom = dx * ey - dy * ex;
    if (fabsf(denom) < 1e-6f) continue;

    float t = ((ax - rx) * ey - (ay - ry) * ex) / denom;
    float u = ((ax - rx) * dy - (ay - ry) * dx) / denom;

    if (t > 0.0f && u >= 0.0f && u <= 1.0f) {
      if (best < 0.0f || t < best) best = t;
    }
  }
  return best;
}

bool insideMap(float x, float y) {
  if (x < 0.0f || x > MAP_WIDTH_MM || y < 0.0f || y > MAP_HEIGHT_MM) return false;
  float notchLow  = MAP_HEIGHT_MM / 2.0f - NOTCH_HEIGHT_MM / 2.0f;
  float notchHigh = MAP_HEIGHT_MM / 2.0f + NOTCH_HEIGHT_MM / 2.0f;
  if (y > notchLow && y < notchHigh) {
    if (x < NOTCH_INSET_MM) return false;
    if (x > MAP_WIDTH_MM - NOTCH_INSET_MM) return false;
  }
  return true;
}

// =====================================================================================
// LD14P packet parsing / UART framing (unchanged)
// =====================================================================================

#define PACKET_SIZE   47
#define PACKET_HEADER 0x54
#define PACKET_VERLEN 0x2C

uint8_t crc8_ld(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x4D) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static inline uint16_t rd16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

bool parsePacket(const uint8_t *buf, LidarPacket &out) {
  if (buf[0] != PACKET_HEADER || buf[1] != PACKET_VERLEN) return false;
  if (crc8_ld(buf, PACKET_SIZE - 1) != buf[PACKET_SIZE - 1]) return false;

  out.speed      = rd16(buf + 2);
  out.startAngle = rd16(buf + 4);
  for (int i = 0; i < 12; i++) {
    const uint8_t *p = buf + 6 + i * 3;
    out.distance_mm[i] = rd16(p);
    out.intensity[i]   = p[2];
  }
  out.endAngle  = rd16(buf + 42);
  out.timestamp = rd16(buf + 44);
  return true;
}

uint8_t rxBuf[PACKET_SIZE];
uint8_t rxIdx = 0;
enum RxState { WAIT_HEADER, WAIT_VERLEN, READ_BODY };
RxState rxState = WAIT_HEADER;

void handlePacket(const LidarPacket &pkt);

void pollLidarSerial() {
  while (LIDAR_SERIAL.available()) {
    uint8_t b = LIDAR_SERIAL.read();
    switch (rxState) {
      case WAIT_HEADER:
        if (b == PACKET_HEADER) { rxBuf[0] = b; rxIdx = 1; rxState = WAIT_VERLEN; }
        break;
      case WAIT_VERLEN:
        if (b == PACKET_VERLEN) { rxBuf[1] = b; rxIdx = 2; rxState = READ_BODY; }
        else if (b == PACKET_HEADER) { rxBuf[0] = b; rxIdx = 1; }
        else { rxState = WAIT_HEADER; }
        break;
      case READ_BODY:
        rxBuf[rxIdx++] = b;
        if (rxIdx >= PACKET_SIZE) {
          LidarPacket pkt;
          if (parsePacket(rxBuf, pkt)) handlePacket(pkt);
          rxState = WAIT_HEADER;
          rxIdx = 0;
        }
        break;
    }
  }
}

// =====================================================================================
// Point weighting (unchanged)
// =====================================================================================

float pointWeight(uint16_t distance_mm, uint8_t intensity) {
  if (distance_mm < MIN_VALID_MM || distance_mm > MAX_VALID_MM) return 0.0f;
  float wConf = intensity / 255.0f;

  float mid    = (MIN_VALID_MM + MAX_VALID_MM) * 0.35f;
  float spread = MAX_VALID_MM * 0.6f;
  float wRange = 1.0f - fabsf((float)distance_mm - mid) / spread;
  if (wRange < 0.05f) wRange = 0.05f;

  return wConf * wRange;
}

// =====================================================================================
// Revolution binning
// =====================================================================================

float binSumDist[NUM_BINS];
float binSumWeight[NUM_BINS];
int32_t lastStartAngle = -1;

void resetScanBins() {
  for (int i = 0; i < NUM_BINS; i++) { binSumDist[i] = 0; binSumWeight[i] = 0; }
}

// =====================================================================================
// Pose scoring — shared by the position grid search and the rotation search
// =====================================================================================

// Precomputes world-frame ray directions for a fixed theta, so the (x,y) grid search
// below doesn't redo cosf/sinf for every candidate cell.
void computeDirs(const float *localAngle, float theta, float *dirX, float *dirY, int n) {
  for (int i = 0; i < n; i++) {
    float rad = radians(localAngle[i] + theta);
    dirX[i] = cosf(rad);
    dirY[i] = sinf(rad);
  }
}

Pose gridSearch(float cx, float cy, float halfRangeX, float halfRangeY, float step,
                const float *meas, const float *weight,
                const float *dirX, const float *dirY, int n) {
  Pose best = { cx, cy, 1e18f };
  for (float x = cx - halfRangeX; x <= cx + halfRangeX; x += step) {
    for (float y = cy - halfRangeY; y <= cy + halfRangeY; y += step) {
      if (!insideMap(x, y)) continue;

      float cost = 0.0f, wsum = 0.0f;
      for (int i = 0; i < n; i++) {
        float expected = rayDistanceToPolygon(x, y, dirX[i], dirY[i]);
        if (expected < 0.0f) expected = MAX_VALID_MM;
        float err = meas[i] - expected;
        cost += weight[i] * err * err;
        wsum += weight[i];
      }
      if (wsum <= 0.0f) continue;
      cost /= wsum;

      if (cost < best.cost) { best.cost = cost; best.x = x; best.y = y; }
    }
  }
  return best;
}

// Same weighted-MSE metric as gridSearch's inner loop, but for a fixed (x,y) and a
// varying theta — recomputes direction per call since it's only used for 1D sweeps.
float poseCost(float x, float y, float theta,
               const float *localAngle, const float *meas, const float *weight, int n) {
  float cost = 0.0f, wsum = 0.0f;
  for (int i = 0; i < n; i++) {
    float rad = radians(localAngle[i] + theta);
    float expected = rayDistanceToPolygon(x, y, cosf(rad), sinf(rad));
    if (expected < 0.0f) expected = MAX_VALID_MM;
    float err = meas[i] - expected;
    cost += weight[i] * err * err;
    wsum += weight[i];
  }
  return (wsum > 0.0f) ? (cost / wsum) : 1e18f;
}

float searchTheta(float x, float y, float center, float halfRange, float step,
                   const float *localAngle, const float *meas, const float *weight, int n,
                   float *outCost) {
  float best = center, bestCost = 1e18f;
  for (float t = center - halfRange; t <= center + halfRange; t += step) {
    float tw = fmodf(t + 360.0f, 360.0f);
    float c = poseCost(x, y, tw, localAngle, meas, weight, n);
    if (c < bestCost) { bestCost = c; best = tw; }
  }
  *outCost = bestCost;
  return best;
}

// =====================================================================================
// Temporal fusion of per-revolution (x, y, theta) fixes
// =====================================================================================

bool  fusionInitialized = false;
float fusedX     = MAP_WIDTH_MM / 2.0f;
float fusedY     = MAP_HEIGHT_MM / 2.0f;
float fusedTheta = 0.0f;   // see 180-degree ambiguity caveat at top of file

void finalizeRevolution() {
  float localAngle[NUM_BINS], meas[NUM_BINS], weight[NUM_BINS];
  int n = 0;
  const float binWidthDeg = 360.0f / NUM_BINS;

  for (int b = 0; b < NUM_BINS; b++) {
    if (binSumWeight[b] <= 0.0f) continue;
    localAngle[n] = b * binWidthDeg + binWidthDeg * 0.5f;
    meas[n]       = binSumDist[b] / binSumWeight[b];
    weight[n]     = binSumWeight[b];
    n++;
  }

  if (n < MIN_RAYS_FOR_FIX) {
    Serial.println("scan: not enough valid returns this revolution, skipping fix");
    return;
  }

  float dirX[NUM_BINS], dirY[NUM_BINS];
  float finalX, finalY, finalTheta, finalCost;

  if (!fusionInitialized) {
    // First fix: no prior heading/position, so sweep all of theta jointly with a
    // coarse position grid. Slow (one-off, robot should be roughly stationary).
    float bestTheta = 0.0f, bestX = fusedX, bestY = fusedY, bestCost = 1e18f;

    for (float t = 0.0f; t < 360.0f; t += THETA_BOOT_STEP_DEG) {
      computeDirs(localAngle, t, dirX, dirY, n);
      Pose s1 = gridSearch(MAP_WIDTH_MM / 2.0f, MAP_HEIGHT_MM / 2.0f,
                            MAP_WIDTH_MM / 2.0f, MAP_HEIGHT_MM / 2.0f,
                            THETA_BOOT_POS_STEP_MM, meas, weight, dirX, dirY, n);
      if (s1.cost < bestCost) { bestCost = s1.cost; bestX = s1.x; bestY = s1.y; bestTheta = t; }
    }

    computeDirs(localAngle, bestTheta, dirX, dirY, n);
    Pose s2 = gridSearch(bestX, bestY, STAGE2_HALFRANGE_MM, STAGE2_HALFRANGE_MM,
                          STAGE2_STEP_MM, meas, weight, dirX, dirY, n);
    Pose s3 = gridSearch(s2.x, s2.y, STAGE3_HALFRANGE_MM, STAGE3_HALFRANGE_MM,
                          STAGE3_STEP_MM, meas, weight, dirX, dirY, n);

    float tCost;
    float theta1 = searchTheta(s3.x, s3.y, bestTheta, THETA_REFINE1_HALF_DEG, THETA_REFINE1_STEP_DEG,
                                localAngle, meas, weight, n, &tCost);
    float theta2 = searchTheta(s3.x, s3.y, theta1, THETA_REFINE2_HALF_DEG, THETA_REFINE2_STEP_DEG,
                                localAngle, meas, weight, n, &tCost);

    finalX = s3.x; finalY = s3.y; finalTheta = theta2; finalCost = tCost;
    fusedX = finalX; fusedY = finalY; fusedTheta = finalTheta;
    fusionInitialized = true;

  } else {
    // Steady state: assume we're close to the last fix, refine position then heading.
    computeDirs(localAngle, fusedTheta, dirX, dirY, n);
    Pose s2 = gridSearch(fusedX, fusedY, STAGE2_HALFRANGE_MM, STAGE2_HALFRANGE_MM,
                          STAGE2_STEP_MM, meas, weight, dirX, dirY, n);
    Pose s3 = gridSearch(s2.x, s2.y, STAGE3_HALFRANGE_MM, STAGE3_HALFRANGE_MM,
                          STAGE3_STEP_MM, meas, weight, dirX, dirY, n);

    float tCost;
    float theta1 = searchTheta(s3.x, s3.y, fusedTheta, THETA_REFINE1_HALF_DEG, THETA_REFINE1_STEP_DEG,
                                localAngle, meas, weight, n, &tCost);
    float theta2 = searchTheta(s3.x, s3.y, theta1, THETA_REFINE2_HALF_DEG, THETA_REFINE2_STEP_DEG,
                                localAngle, meas, weight, n, &tCost);

    finalX = s3.x; finalY = s3.y; finalTheta = theta2; finalCost = tCost;

    float qLocal = 1.0f / (1.0f + finalCost / QUALITY_SCALE);
    float alpha = FUSION_BASE_ALPHA * qLocal;
    if (alpha < FUSION_MIN_ALPHA) alpha = FUSION_MIN_ALPHA;
    if (alpha > FUSION_MAX_ALPHA) alpha = FUSION_MAX_ALPHA;

    fusedX += alpha * (finalX - fusedX);
    fusedY += alpha * (finalY - fusedY);
    fusedTheta = fmodf(fusedTheta + alpha * angleDiffDeg(finalTheta, fusedTheta) + 360.0f, 360.0f);
  }

  float quality = 1.0f / (1.0f + finalCost / QUALITY_SCALE);
  float outX = fusedX - MAP_WIDTH_MM / 2.0f;
  float outY = fusedY - MAP_HEIGHT_MM / 2.0f;

  Serial.printf("pos=(%.1f, %.1f) mm from center | heading=%.1f deg | quality=%.2f | rays=%d\n",
                outX, outY, fusedTheta, quality, n);
}

// =====================================================================================
// Packet handling / PWM speed trim (unchanged)
// =====================================================================================

uint32_t pwmTrimCounter = 0;

void handlePacket(const LidarPacket &pkt) {
  pwmTrimCounter++;
  if (pwmTrimCounter % PWM_TRIM_PERIOD == 0) {
    float actualHz = pkt.speed / 360.0f;
    if (actualHz < TARGET_SCAN_HZ - 0.3f && pwmDuty < 255) pwmDuty++;
    else if (actualHz > TARGET_SCAN_HZ + 0.3f && pwmDuty > 0) pwmDuty--;
    analogWrite(LIDAR_PWM_PIN, pwmDuty);
  }

  if (lastStartAngle >= 0 && (int32_t)pkt.startAngle < lastStartAngle) {
    finalizeRevolution();
    resetScanBins();
  }
  lastStartAngle = pkt.startAngle;

  int32_t angleStep = (int32_t)pkt.endAngle - (int32_t)pkt.startAngle;
  if (angleStep < 0) angleStep += 36000;

  const float binWidthDeg = 360.0f / NUM_BINS;
  for (int i = 0; i < 12; i++) {
    int32_t angleHundredths = (int32_t)pkt.startAngle + (angleStep * i) / 11;
    angleHundredths %= 36000;
    float localAngleDeg = angleHundredths / 100.0f;

    if (angleBlockedByPole(localAngleDeg)) continue;

    float w = pointWeight(pkt.distance_mm[i], pkt.intensity[i]);
    if (w <= 0.0f) continue;

    int bin = ((int)(localAngleDeg / binWidthDeg)) % NUM_BINS;
    binSumDist[bin]   += w * pkt.distance_mm[i];
    binSumWeight[bin] += w;
  }
}

// =====================================================================================
// Arduino entry points
// =====================================================================================

void setup() {
  Serial.begin(DEBUG_BAUD);
  LIDAR_SERIAL.begin(LIDAR_BAUD);

  pinMode(LIDAR_PWM_PIN, OUTPUT);
  analogWriteResolution(8);
  analogWriteFrequency(LIDAR_PWM_PIN, LIDAR_PWM_FREQ_HZ);
  analogWrite(LIDAR_PWM_PIN, pwmDuty);

  resetScanBins();
}

void loop() {
  pollLidarSerial();
}
