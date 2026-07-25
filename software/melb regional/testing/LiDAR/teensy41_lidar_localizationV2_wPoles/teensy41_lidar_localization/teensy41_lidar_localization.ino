/* =====================================================================================
   Teensy 4.1  +  LDROBOT / Waveshare LD14P 2D LiDAR
   Ray-cast localization inside a known concave rectangle, with a "mock" weighted
   sensor-fusion pass over the LiDAR returns and a stubbed-in gyro heading input.

   ------------------------------------------------------------------------------------
   WHAT THIS DOES
   ------------------------------------------------------------------------------------
   1. Streams and parses LD14P UART packets (230400-8N1, CRC8 poly 0x4D verified).
   2. Drives the LD14P's PWM speed-control input in a closed loop, trimmed off the
      LiDAR's own reported "speed" field, to push it toward its max scan rate.
   3. Bins one full 360 deg revolution of returns into 5 deg buckets, weighting each
      bucket by (a) the LD14P's reported per-point intensity/confidence byte and
      (b) a simple near/far range-accuracy heuristic. This is the "mock sensor
      fusion" weighting for the raw ranging data.
   4. Ray-casts against the known map polygon (your rectangle + two notches) from
      candidate (x,y) positions, coarse-to-fine grid search, to find the pose whose
      simulated scan best matches the real one (weighted least squares).
   5. Temporally fuses that per-revolution pose fix into a running estimate with an
      adaptive-gain filter -- low-confidence scans barely move the estimate,
      high-confidence scans (clean notch detections) snap to it quickly.
   6. Tracks a single `robotHeadingDeg` variable, refreshed once per revolution by
      a `updateGyroHeading()` stub -- wire in your actual IMU driver there. It's
      only used to rotate LiDAR-local ray angles into the map's world frame; there's
      no accelerometer here, so there's nothing to integrate for *position*, only
      rotation, per your description.

   ------------------------------------------------------------------------------------
   MAP GEOMETRY -- FROM YOUR DRAWING
   ------------------------------------------------------------------------------------
   Outer rectangle: 2430 mm x 1820 mm.
   Both SHORT edges (the 1820 mm left/right walls) have a rectangular notch cut into
   the interior, centered on the vertical midline: 470 mm tall, 226 mm deep.

   NOTE: your drawing dimensions the notch depth as 226 mm; your written description
   said 26.6 cm (266 mm). I went with the drawing (226 mm, see NOTCH_INSET_MM below)
   since it's the more precise of the two sources -- change the #define if that's
   wrong for your actual field.

          (0,1820)------------------------------------------------(2430,1820)
             |                                                          |
             |                                                          |
        (0,1145)---(226,1145)                            (2204,675)---(2430,675)
             |            |                                    |            |
             |            |                                    |            |
         (0,675)----(226,675)                            (2204,1145)--(2430,1145)
             |                                                          |
             |                                                          |
           (0,0)-------------------------------------------------(2430,0)

   Internally the map is stored with the origin at the BOTTOM-LEFT outer corner
   (matches the "0" tick marks on your dimension lines). The final reported
   position is re-centered to the rectangle's center, per your request.

   ------------------------------------------------------------------------------------
   WIRING (verify against your specific LD14P breakout -- connector pinouts / colors
   differ between vendors, e.g. Waveshare's kit vs a bare LDROBOT connector)
   ------------------------------------------------------------------------------------
   LD14P VCC        -> board 5V supply (check your module's exact voltage rating)
   LD14P GND        -> Teensy GND (common ground with the 5V supply too)
   LD14P TX         -> Teensy RX2 (pin 7)      [LD14P UART is one-way, 3.3V logic]
   LD14P PWM/SPEED  -> Teensy pin 2 (PWM out)  [only if using external speed control;
                                                 tie this pin to GND instead if you
                                                 want the LiDAR's default 6 Hz internal
                                                 speed control and don't want this
                                                 sketch driving it]

   ------------------------------------------------------------------------------------
   HONEST CAVEATS
   ------------------------------------------------------------------------------------
   - This is a from-scratch, un-benchmarked-on-real-hardware educational sketch, not
     a vetted localization stack. Treat it as a starting point.
   - TARGET_SCAN_HZ is set to 8 Hz, the documented top of the LD14P's external-control
     range (2-8 Hz, 6 Hz default). The datasheet notes actual speed varies motor-to-motor
     at a given duty cycle, which is why this sketch closes the loop on the LiDAR's own
     reported speed field rather than just writing a fixed PWM duty cycle and hoping.
   - Four support poles mounted to the chassis flank the LD14P and permanently block
     fixed angular slivers of its view (see POLE_* below) -- those angles are excluded
     from every scan before they ever reach the binning/localization code.
   - The point-weighting curve and the grid-search resolution are simple, tunable
     heuristics ("mock" fusion as you asked for), not manufacturer accuracy curves.
   - A real deployment would want something more robust than grid search (e.g. a
     particle filter / AMCL) to handle multi-modal ambiguity and sensor noise -- the
     two notches are what make this shape localizable at all (a plain rectangle is
     symmetric and would leave you with a 2-4x pose ambiguity).
   ===================================================================================== */

#include <math.h>
#include "lidar_types.h"   // LidarPacket, Pose -- see that file for why it's split out

// =====================================================================================
// -------------------------------  CONFIG / TUNABLES  ---------------------------------
// =====================================================================================

// ---- Serial ports ----
#define LIDAR_SERIAL      Serial3      // LD14P TX -> Teensy RX2 (pin 7)
#define LIDAR_BAUD        230400
#define DEBUG_BAUD        115200

// ---- LD14P speed control (external PWM) ----
#define LIDAR_PWM_PIN      14           // Teensy pin driving the LD14P's speed-ctrl input
#define LIDAR_PWM_FREQ_HZ  1000        // datasheet: 500 Hz - 1.5 kHz, 1 kHz recommended
#define TARGET_SCAN_HZ      8.0f       // LD14P's own datasheet: default 6Hz, external-control
                                        // range is 2-8Hz -- 8Hz is genuinely its max, not the
                                        // ~13Hz some sibling LD-series units support
#define PWM_TRIM_PERIOD    20          // only trim duty every N packets (avoid hunting)
uint8_t pwmDuty = 220;                 // 0-255, start high since we're aiming for max speed

// ---- Known map geometry (mm), origin = bottom-left outer corner ----
#define MAP_WIDTH_MM       2430.0f
#define MAP_HEIGHT_MM      1820.0f
#define NOTCH_INSET_MM      226.0f     // horizontal depth of each notch (see caveat above)
#define NOTCH_HEIGHT_MM     470.0f     // vertical extent of each notch, centered mid-height

// ---- Point validity / weighting ----
#define MIN_VALID_MM        50.0f      // discard returns inside the blind zone
#define MAX_VALID_MM     12000.0f      // discard implausibly-far returns
#define MIN_RAYS_FOR_FIX      8        // need at least this many valid bins to try a fix

// ---- Scan binning ----
#define NUM_BINS              72       // 5 deg buckets across 360 deg

// ---- Grid-search localization (coarse -> fine) ----
#define STAGE1_STEP_MM       100.0f
#define STAGE2_HALFRANGE_MM  150.0f
#define STAGE2_STEP_MM        20.0f
#define STAGE3_HALFRANGE_MM   25.0f
#define STAGE3_STEP_MM         4.0f
#define QUALITY_SCALE      160000.0f   // ~ (400mm)^2, tunes cost -> quality mapping

// ---- Temporal (mock) sensor fusion of successive pose fixes ----
#define FUSION_BASE_ALPHA      0.6f
#define FUSION_MIN_ALPHA       0.05f
#define FUSION_MAX_ALPHA       0.9f

// ---- Fixed chassis-mounted poles that permanently block part of the LiDAR's view ----
// These are bolted to the robot, not the room, so they always block the same LOCAL
// (LiDAR-frame) angles regardless of where the robot is or which way it's facing --
// completely separate from robot heading/world-frame math. Per LDROBOT's own LD14P
// datasheet, local angle 0 deg is DEFINED as the direction from the rotation center
// toward the LiDAR's internal drive motor -- i.e. "the motor side" already is local
// angle 0/360 in the data you parse. Default below places the 4 poles symmetrically
// about that reference, two per side, 45 deg apart from each other and from center --
// ADJUST poleCenterAngleDeg[] if your actual mounting differs.
#define POLE_COUNT           4
#define ROTOR_RADIUS_MM      30.0f   // rough radius of the LD14P's spinning head, from its
                                      // ~59.8mm overall housing width -- MEASURE YOUR UNIT;
                                      // this number directly sets how wide each blocked
                                      // window is computed to be
#define POLE_STANDOFF_MM     10.0f   // distance from the spinning edge to the near face
                                      // of each pole, as you measured it
#define POLE_WIDTH_MM          7.0f
#define POLE_DISTANCE_MM     (ROTOR_RADIUS_MM + POLE_STANDOFF_MM) // from rotation center,
                                                                    // i.e. from angle origin

const float poleCenterAngleDeg[POLE_COUNT] = { -67.5f, -22.5f, 22.5f, 67.5f }; // vs motor side (0 deg)

// Half-angle subtended by one pole (width POLE_WIDTH_MM) at range POLE_DISTANCE_MM.
// A pole is opaque, so it blocks this whole angular sliver at every range behind it --
// no need to also gate on returned distance.
const float poleHalfWidthDeg = degrees(atanf((POLE_WIDTH_MM / 2.0f) / POLE_DISTANCE_MM));

// Smallest signed angular difference a-b, wrapped into (-180, 180].
float angleDiffDeg(float a, float b) {
  float d = fmodf(a - b + 540.0f, 360.0f) - 180.0f;
  return d;
}

bool angleBlockedByPole(float localAngleDeg) {
  for (int i = 0; i < POLE_COUNT; i++) {
    if (fabsf(angleDiffDeg(localAngleDeg, poleCenterAngleDeg[i])) <= poleHalfWidthDeg) return true;
  }
  return false;
}

// =====================================================================================
// ------------------------------  MAP POLYGON GEOMETRY  -------------------------------
// =====================================================================================

const float mapVertices[12][2] = {
  { 0.0f,                                MAP_HEIGHT_MM },                                  // 0 top-left
  { 0.0f,                                MAP_HEIGHT_MM/2.0f + NOTCH_HEIGHT_MM/2.0f },       // 1 left, notch top
  { NOTCH_INSET_MM,                      MAP_HEIGHT_MM/2.0f + NOTCH_HEIGHT_MM/2.0f },       // 2 notch inner-top
  { NOTCH_INSET_MM,                      MAP_HEIGHT_MM/2.0f - NOTCH_HEIGHT_MM/2.0f },       // 3 notch inner-bottom
  { 0.0f,                                MAP_HEIGHT_MM/2.0f - NOTCH_HEIGHT_MM/2.0f },       // 4 left, notch bottom
  { 0.0f,                                0.0f },                                            // 5 bottom-left
  { MAP_WIDTH_MM,                        0.0f },                                            // 6 bottom-right
  { MAP_WIDTH_MM,                        MAP_HEIGHT_MM/2.0f - NOTCH_HEIGHT_MM/2.0f },       // 7 right, notch bottom
  { MAP_WIDTH_MM - NOTCH_INSET_MM,       MAP_HEIGHT_MM/2.0f - NOTCH_HEIGHT_MM/2.0f },       // 8 notch inner-bottom
  { MAP_WIDTH_MM - NOTCH_INSET_MM,       MAP_HEIGHT_MM/2.0f + NOTCH_HEIGHT_MM/2.0f },       // 9 notch inner-top
  { MAP_WIDTH_MM,                        MAP_HEIGHT_MM/2.0f + NOTCH_HEIGHT_MM/2.0f },       // 10 right, notch top
  { MAP_WIDTH_MM,                        MAP_HEIGHT_MM }                                    // 11 top-right
};
#define NUM_EDGES 12

// Ray (origin rx,ry ; direction dx,dy, need not be normalized) vs. each map edge.
// Returns distance to the nearest edge intersection along the ray, or -1 if none
// (shouldn't happen for a ray cast from inside a closed polygon).
float rayDistanceToPolygon(float rx, float ry, float dx, float dy) {
  float best = -1.0f;
  for (int i = 0; i < NUM_EDGES; i++) {
    float ax = mapVertices[i][0];
    float ay = mapVertices[i][1];
    int j = (i + 1) % NUM_EDGES;
    float ex = mapVertices[j][0] - ax;
    float ey = mapVertices[j][1] - ay;

    float denom = dx * ey - dy * ex;
    if (fabsf(denom) < 1e-6f) continue; // parallel

    float t = ((ax - rx) * ey - (ay - ry) * ex) / denom; // distance along the ray
    float u = ((ax - rx) * dy - (ay - ry) * dx) / denom; // fraction along the segment

    if (t > 0.0f && u >= 0.0f && u <= 1.0f) {
      if (best < 0.0f || t < best) best = t;
    }
  }
  return best;
}

// True if (x,y) is inside the playable area -- i.e. not inside one of the two
// wall-recess cutouts. Needed because the map is concave: the bounding box alone
// isn't enough to know a candidate pose is physically valid.
bool insideMap(float x, float y) {
  if (x < 0.0f || x > MAP_WIDTH_MM || y < 0.0f || y > MAP_HEIGHT_MM) return false;
  float notchLow  = MAP_HEIGHT_MM / 2.0f - NOTCH_HEIGHT_MM / 2.0f;
  float notchHigh = MAP_HEIGHT_MM / 2.0f + NOTCH_HEIGHT_MM / 2.0f;
  if (y > notchLow && y < notchHigh) {
    if (x < NOTCH_INSET_MM) return false;                    // inside left cutout
    if (x > MAP_WIDTH_MM - NOTCH_INSET_MM) return false;      // inside right cutout
  }
  return true;
}

// =====================================================================================
// -----------------------------  LD14P PACKET DEFINITION  -----------------------------
// =====================================================================================

#define PACKET_SIZE   47
#define PACKET_HEADER 0x54
#define PACKET_VERLEN 0x2C   // fixed: 12 points/packet, packet type 1

// CRC8, polynomial 0x4D, MSB-first, init 0, no reflect, no xor-out -- matches the
// LD14P/LD19/LD06 family protocol. Computed bitwise here (rather than pasting a
// 256-entry lookup table) -- verified against the vendor SDK's known table values.
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
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8); // little-endian, LSB first
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

// =====================================================================================
// -----------------------------  UART FRAMING STATE MACHINE  --------------------------
// =====================================================================================

uint8_t rxBuf[PACKET_SIZE];
uint8_t rxIdx = 0;
enum RxState { WAIT_HEADER, WAIT_VERLEN, READ_BODY };
RxState rxState = WAIT_HEADER;

void handlePacket(const LidarPacket &pkt); // fwd decl

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
// ------------------------  POINT WEIGHTING ("MOCK" FUSION, PT 1)  --------------------
// =====================================================================================

// How much to trust one LiDAR return, based on:
//   (a) its reported intensity/confidence byte (0-255) -- LD14P's own signal-quality
//       proxy, and
//   (b) a simple near/far range de-weighting, since DTOF returns tend to be least
//       trustworthy right at the blind-zone edge and out past the module's
//       comfortable range. This is a heuristic, not a datasheet accuracy curve --
//       tune `mid`/`spread` against your own unit if you have ground truth to check.
float pointWeight(uint16_t distance_mm, uint8_t intensity) {
  if (distance_mm < MIN_VALID_MM || distance_mm > MAX_VALID_MM) return 0.0f;
  float wConf = intensity / 255.0f;

  float mid    = (MIN_VALID_MM + MAX_VALID_MM) * 0.35f;
  float spread = MAX_VALID_MM * 0.6f;
  float wRange = 1.0f - fabsf((float)distance_mm - mid) / spread;
  if (wRange < 0.05f) wRange = 0.05f; // never fully zero out a valid return

  return wConf * wRange;
}

// =====================================================================================
// ---------------------------------  REVOLUTION BINNING  ------------------------------
// =====================================================================================

float binSumDist[NUM_BINS];
float binSumWeight[NUM_BINS];
int32_t lastStartAngle = -1;

void resetScanBins() {
  for (int i = 0; i < NUM_BINS; i++) { binSumDist[i] = 0; binSumWeight[i] = 0; }
}

// =====================================================================================
// -----------------------------------  GYRO HEADING  -----------------------------------
// =====================================================================================

// Continuously-integrated heading in degrees, 0 = LiDAR's 0-angle ("front") direction
// in the map frame, increasing clockwise (matches the LD14P's own angle convention).
// Refreshed once per revolution, right when a new pose fix is about to be computed.
float robotHeadingDeg = 0.0f;

void updateGyroHeading() {
  // TODO: wire in your actual IMU/gyro driver here, e.g.:
  //   robotHeadingDeg = myIMU.getYawDegrees();
  // Left as a stub per your request -- this sketch only *consumes* the variable.
}

// =====================================================================================
// -----------------------------  GRID-SEARCH LOCALIZATION  ----------------------------
// =====================================================================================

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
        if (expected < 0.0f) expected = MAX_VALID_MM; // shouldn't happen inside the map
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

// =====================================================================================
// -------------------------  TEMPORAL FUSION OF POSE FIXES (PT 2)  --------------------
// =====================================================================================

bool  fusionInitialized = false;
float fusedX = MAP_WIDTH_MM / 2.0f;
float fusedY = MAP_HEIGHT_MM / 2.0f;

void finalizeRevolution() {
  // Gyro is only sampled here -- "updated each time the calculations are made" --
  // and only used to rotate LiDAR-local ray angles into the map's world frame.
  updateGyroHeading();

  float meas[NUM_BINS], weight[NUM_BINS], dirX[NUM_BINS], dirY[NUM_BINS];
  int n = 0;
  const float binWidthDeg = 360.0f / NUM_BINS;

  for (int b = 0; b < NUM_BINS; b++) {
    if (binSumWeight[b] <= 0.0f) continue;
    float avgDist = binSumDist[b] / binSumWeight[b];
    float localAngleDeg = b * binWidthDeg + binWidthDeg * 0.5f;
    float worldAngleRad = radians(localAngleDeg + robotHeadingDeg);

    meas[n]   = avgDist;
    weight[n] = binSumWeight[b];
    dirX[n]   = cosf(worldAngleRad);
    dirY[n]   = sinf(worldAngleRad);
    n++;
  }

  if (n < MIN_RAYS_FOR_FIX) {
    Serial.println("scan: not enough valid returns this revolution, skipping fix");
    return;
  }

  // Coarse -> fine grid search.
  Pose s1 = gridSearch(MAP_WIDTH_MM / 2.0f, MAP_HEIGHT_MM / 2.0f,
                        MAP_WIDTH_MM / 2.0f, MAP_HEIGHT_MM / 2.0f,
                        STAGE1_STEP_MM, meas, weight, dirX, dirY, n);
  Pose s2 = gridSearch(s1.x, s1.y, STAGE2_HALFRANGE_MM, STAGE2_HALFRANGE_MM,
                        STAGE2_STEP_MM, meas, weight, dirX, dirY, n);
  Pose s3 = gridSearch(s2.x, s2.y, STAGE3_HALFRANGE_MM, STAGE3_HALFRANGE_MM,
                        STAGE3_STEP_MM, meas, weight, dirX, dirY, n);

  float quality = 1.0f / (1.0f + s3.cost / QUALITY_SCALE); // ~0..1

  if (!fusionInitialized) {
    fusedX = s3.x;
    fusedY = s3.y;
    fusionInitialized = true;
  } else {
    float alpha = FUSION_BASE_ALPHA * quality;
    if (alpha < FUSION_MIN_ALPHA) alpha = FUSION_MIN_ALPHA;
    if (alpha > FUSION_MAX_ALPHA) alpha = FUSION_MAX_ALPHA;
    fusedX += alpha * (s3.x - fusedX);
    fusedY += alpha * (s3.y - fusedY);
  }

  float outX = fusedX - MAP_WIDTH_MM / 2.0f;   // re-center: (0,0) = rectangle center
  float outY = fusedY - MAP_HEIGHT_MM / 2.0f;

  Serial.printf("pos=(%.1f, %.1f) mm from center | heading=%.1f deg | quality=%.2f | rays=%d\n",
                outX, outY, robotHeadingDeg, quality, n);
}

// =====================================================================================
// --------------------------  PACKET HANDLING / PWM SPEED TRIM  -----------------------
// =====================================================================================

uint32_t pwmTrimCounter = 0;

void handlePacket(const LidarPacket &pkt) {
  // --- closed-loop trim toward max scan speed, using the LiDAR's own reported speed ---
  pwmTrimCounter++;
  if (pwmTrimCounter % PWM_TRIM_PERIOD == 0) {
    float actualHz = pkt.speed / 360.0f;
    if (actualHz < TARGET_SCAN_HZ - 0.3f && pwmDuty < 255) pwmDuty++;
    else if (actualHz > TARGET_SCAN_HZ + 0.3f && pwmDuty > 0) pwmDuty--;
    analogWrite(LIDAR_PWM_PIN, pwmDuty);
  }

  // --- revolution-wrap detection (start angle rolling back over 0/360) ---
  if (lastStartAngle >= 0 && (int32_t)pkt.startAngle < lastStartAngle) {
    finalizeRevolution();
    resetScanBins();
  }
  lastStartAngle = pkt.startAngle;

  // --- bin this packet's 12 points ---
  int32_t angleStep = (int32_t)pkt.endAngle - (int32_t)pkt.startAngle;
  if (angleStep < 0) angleStep += 36000; // wrapped past 360.00 deg within the packet

  const float binWidthDeg = 360.0f / NUM_BINS;
  for (int i = 0; i < 12; i++) {
    int32_t angleHundredths = (int32_t)pkt.startAngle + (angleStep * i) / 11;
    angleHundredths %= 36000;
    float localAngleDeg = angleHundredths / 100.0f;

    // Chassis poles physically block this angular sliver at every revolution --
    // whatever came back here (if anything) is a pole reflection, not map data.
    if (angleBlockedByPole(localAngleDeg)) continue;

    float w = pointWeight(pkt.distance_mm[i], pkt.intensity[i]);
    if (w <= 0.0f) continue;

    int bin = ((int)(localAngleDeg / binWidthDeg)) % NUM_BINS;
    binSumDist[bin]   += w * pkt.distance_mm[i];
    binSumWeight[bin] += w;
  }
}

// =====================================================================================
// ------------------------------------  ARDUINO ENTRY  --------------------------------
// =====================================================================================

void setup() {
  Serial.begin(DEBUG_BAUD);
  LIDAR_SERIAL.begin(LIDAR_BAUD);

  pinMode(LIDAR_PWM_PIN, OUTPUT);
  analogWriteResolution(8);
  analogWriteFrequency(LIDAR_PWM_PIN, LIDAR_PWM_FREQ_HZ);
  analogWrite(LIDAR_PWM_PIN, pwmDuty); // start near-max duty; closed loop trims from here

  resetScanBins();
}

void loop() {
  pollLidarSerial();
  // Everything else (binning, gyro sampling, localization, fusion, printing) happens
  // event-driven inside handlePacket()/finalizeRevolution() as packets arrive.
}
