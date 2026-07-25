#include "ball_tracking.h"

#include "drivebase.h"
#include "imu.h"
#include "robot_config.h"

// ============================================================
// Ball Tracking (bit-banged, CS-framed SPI-style link to OpenMV)
// ============================================================
//
// The OpenMV cam drives SCK + MOSI (SPI mode 0, MSB-first) AND a CS line
// that goes low for the duration of each 8-byte packet and back high
// once it's done. CS framing means the receiver always knows exactly
// where a packet starts/ends instead of having to search for a sync
// byte inside a continuous, unframed stream.
//
// WIRING:
//   Teensy pin 13 (SCK)  <- OpenMV SPI SCLK
//   Teensy pin 11 (MOSI) <- OpenMV SPI MOSI
//   Teensy pin 10 (CS)   <- OpenMV SPI CS (frames each packet)
//   Teensy pin 12 (MISO) -> OpenMV SPI MISO (unused - we never transmit
//                            back - but configured as an output so it
//                            doesn't float/load the bus)
//   Teensy GND             -- OpenMV GND   <-- required, don't skip
//
// PACKET FORMAT (8 bytes, one sent per OpenMV frame, clocked while CS is
// low) - see the OpenMV script for the sending side:
//   byte 0:   sync byte - 0xAA (BALL_SYNC_A) or 0xAB (BALL_SYNC_B),
//             used upstream to distinguish two tracked colors; either
//             is accepted here as "a ball," since only one target is
//             chased
//   byte 1:   detected flag, 0 or 1
//   byte 2-3: angle_deg * 100, signed 16-bit, big-endian
//   byte 4-5: radius_px, unsigned 16-bit, big-endian (off-center pixel
//             distance of the blob from the image center - NOT a
//             physical/world distance)
//   byte 6:   coarse blob-size indicator, 0-255 (bigger = ball looks
//             closer/larger in-frame)
//   byte 7:   checksum = XOR of bytes 0-6
//
// Reception happens in two ISRs:
//   - ballCSISR(), on CHANGE of BALL_SPI_CS_PIN: falling edge resets the
//     bit/byte counters (start of a new packet); rising edge means the
//     transfer is done, and if a full 8 bytes were clocked in, they're
//     copied into ballReadyBuf and ballPacketReady is set for the main
//     code to pick up.
//   - ballClockISR(), on RISING of BALL_SPI_SCK_PIN: bit-bangs one bit
//     of MOSI into the byte currently being assembled.
// processBallPacket() (called every tick from systemTick()) drains
// ballPacketReady, validates the checksum + sync byte, and updates
// latestBallPacket / lastBallPacketMs - the same globals chaseTick()
// already reads via getLatestBallData().

static const int BALL_SPI_SCK_PIN  = 13; // clock in from OpenMV
static const int BALL_SPI_MOSI_PIN = 11; // data in from OpenMV
static const int BALL_SPI_MISO_PIN = 12; // unused output, just kept configured
static const int BALL_SPI_CS_PIN   = 10; // frames each 8-byte packet

static const uint8_t BALL_PACKET_LEN = 8;
static const uint8_t BALL_SYNC_A = 0xAA;
static const uint8_t BALL_SYNC_B = 0xAB;
static const uint8_t BALL_SYNC_C = 0xAC;

struct BallPacket {
  bool detected;
  float angleDeg;   // camera-frame bearing to the ball, see the OpenMV script
  float radiusPx;   // pixel distance of the blob from the image center
  uint8_t sizeByte; // coarse blob-size indicator, 0-255
};

// Written from processBallPacket(), read from chaseTick() via
// getLatestBallData() - never read these two directly, the read isn't
// atomic across fields.
static volatile BallPacket latestBallPacket = {false, 0.0f, 0.0f, 0};
static volatile unsigned long lastBallPacketMs = 0;

// Raw reception state, filled by the two ISRs below.
static volatile uint8_t ballRxByte    = 0;
static volatile uint8_t ballBitCount  = 0;
static volatile uint8_t ballByteIndex = 0;
static volatile uint8_t ballRxBuf[BALL_PACKET_LEN];

static volatile uint8_t ballReadyBuf[BALL_PACKET_LEN];
static volatile bool ballPacketReady = false;

// ---- Ball-chase tuning - EASY ADJUSTMENT KNOBS ----

// Added to the camera's reported bearing to convert it into a
// chassis-relative direction. Calibrate by placing the ball directly in
// front of the chassis (whichever way you want the robot to call
// "forward") and adjusting this until the DEBUG_BALL_CHASE print in
// chaseTick() reads a bearing of about 0 degrees.
//
// This is separate from CAMERA_ROTATION_OFFSET_DEG on the OpenMV side,
// which corrects for the camera's own mounting rotation inside its
// bracket - this one corrects for how that already-corrected bearing
// lines up with the chassis's physical forward axis (e.g. if the camera
// mount itself points a few degrees off from dead-ahead on the chassis).
static const float CAMERA_MOUNT_OFFSET_DEG = 0.0f;

// If no valid packet arrives within this long, treat the ball as lost
// and stop translating rather than drive on stale data.
static const unsigned long BALL_DATA_TIMEOUT_MS = 300;

// Speed ceiling for ball-chasing - keep this well below move()'s usual
// maxSpeed values. The whole point of chaseTick() is to approach the
// ball SLOWLY and under control, not to charge at it.
static const float BALL_CHASE_MAX_SPEED = 0.3f;
static const float BALL_CHASE_MIN_SPEED = 0.20f; // don't bother creeping below this

// Target radiusPx (packet bytes 4-5) the robot drives toward - the
// goal state for chaseTick() is "ball at BALL_TARGET_RADIUS_PX px, at
// 0 degrees bearing" (i.e. centered/aligned and this close). Once
// radiusPx reaches this value the robot stops translating (it keeps
// rotating to hold the bearing at the ball, it just stops driving
// forward). Tune this by watching the printed radiusPx value
// (DEBUG_BALL_CHASE) as you move the real ball closer to and farther
// from the camera.
static const float BALL_TARGET_RADIUS_PX = 30.0f;

// How far past BALL_TARGET_RADIUS_PX (in px) the ball needs to be
// before the robot drives at the full BALL_CHASE_MAX_SPEED. Between
// BALL_TARGET_RADIUS_PX and BALL_TARGET_RADIUS_PX + BALL_CHASE_RAMP_RANGE_PX,
// speed scales down linearly from BALL_CHASE_MAX_SPEED to
// BALL_CHASE_MIN_SPEED, so the robot arrives at the target under
// control instead of stopping abruptly - farther away drives faster,
// closer in slows down. Tune alongside BALL_CHASE_MAX_SPEED/MIN_SPEED.
static const float BALL_CHASE_RAMP_RANGE_PX = 80.0f;

// Fires on every CHANGE of BALL_SPI_CS_PIN. Falling edge = start of a
// new framed transfer (reset the bit/byte counters). Rising edge = the
// transfer is done; if a full 8 bytes were clocked in while CS was low,
// they're copied into ballReadyBuf and ballPacketReady is set for
// processBallPacket() to pick up.
static void ballCSISR() {
  if (digitalReadFast(BALL_SPI_CS_PIN) == LOW) {
    // Falling edge: start of a new framed transfer.
    ballBitCount = 0;
    ballByteIndex = 0;
    ballRxByte = 0;
  } else {
    // Rising edge: transfer done. Only accept a full 8-byte packet.
    if (ballByteIndex == BALL_PACKET_LEN) {
      for (uint8_t i = 0; i < BALL_PACKET_LEN; i++) ballReadyBuf[i] = ballRxBuf[i];
      ballPacketReady = true;
    }
  }
}

// Fires on every rising edge of BALL_SPI_SCK_PIN (SPI mode 0: data is
// valid on the rising edge), sampling one bit of BALL_SPI_MOSI_PIN each
// time and shifting it into a byte. Once 8 bits have arrived (and we're
// still within the current CS-framed packet), the finished byte is
// stored into ballRxBuf. MSB-first shift-in - if bytes ever come out
// bit-reversed, change the shift below to LSB-first instead.
static void ballClockISR() {
  ballRxByte <<= 1;
  if (digitalReadFast(BALL_SPI_MOSI_PIN)) ballRxByte |= 1;
  ballBitCount++;

  if (ballBitCount == 8) {
    if (ballByteIndex < BALL_PACKET_LEN) {
      ballRxBuf[ballByteIndex] = ballRxByte;
      ballByteIndex++;
    }
    ballBitCount = 0;
    ballRxByte = 0;
  }
}

void initBallTracking() {
  // Safe to bring this up before the OpenMV cam powers up - it just
  // sits idle until CS/SCK edges start arriving.
  pinMode(BALL_SPI_MOSI_PIN, INPUT);
  pinMode(BALL_SPI_SCK_PIN, INPUT);
  pinMode(BALL_SPI_CS_PIN, INPUT);
  pinMode(BALL_SPI_MISO_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(BALL_SPI_CS_PIN), ballCSISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BALL_SPI_SCK_PIN), ballClockISR, RISING);
  Serial.println("Ball-tracking CS-framed bit-banged receiver ready");
}

void processBallPacket() {
  if (!ballPacketReady) return;
  ballPacketReady = false;

  uint8_t buf[BALL_PACKET_LEN];
  noInterrupts();
  for (uint8_t i = 0; i < BALL_PACKET_LEN; i++) buf[i] = ballReadyBuf[i];
  interrupts();

  uint8_t checksum = 0;
  for (uint8_t i = 0; i < BALL_PACKET_LEN - 1; i++) checksum ^= buf[i];

  if (checksum != buf[BALL_PACKET_LEN - 1]) {
#ifdef DEBUG_BALL_LINK
    Serial.println("[BALL DROP] checksum mismatch");
#endif
    return; // corrupted packet, drop
  }

  uint8_t sync = buf[0];

  if (sync != BALL_SYNC_A && sync != BALL_SYNC_B) {
#ifdef DEBUG_BALL_LINK
    Serial.println("[BALL DROP] bad sync byte");
#endif
    return;
  }

  latestBallPacket.detected = buf[1] != 0;
  latestBallPacket.angleDeg = ((int16_t)((buf[2] << 8) | buf[3])) / 100.0f;
  latestBallPacket.radiusPx = (float)((uint16_t)((buf[4] << 8) | buf[5]));
  latestBallPacket.sizeByte = buf[6];
  lastBallPacketMs = millis();

#ifdef DEBUG_BALL_LINK
  Serial.print("[BALL PKT OK] sync=");
  Serial.print(sync == BALL_SYNC_A ? "A" : "B");
  Serial.print("  detected=");
  Serial.print(latestBallPacket.detected ? "YES" : "no ");
  Serial.print("  angle=");
  Serial.print(latestBallPacket.angleDeg, 2);
  Serial.print("deg  radius=");
  Serial.print(latestBallPacket.radiusPx, 0);
  Serial.print("px  size=");
  Serial.println(latestBallPacket.sizeByte);
#endif
}

// Safely copies the latest ball packet out of the volatile globals
// processBallPacket() writes into. Call this instead of reading
// latestBallPacket's fields directly, since a multi-field struct read
// isn't guaranteed atomic with respect to the CS/clock ISRs running in
// between.
static void getLatestBallData(BallPacket &out) {
  noInterrupts();
  out.detected = latestBallPacket.detected;
  out.angleDeg = latestBallPacket.angleDeg;
  out.radiusPx = latestBallPacket.radiusPx;
  out.sizeByte = latestBallPacket.sizeByte;
  interrupts();
}

// Runs one tick of "slowly drive towards the ball," using the latest
// packet from the OpenMV cam. Meant to be called repeatedly from a loop
// (see robot.ino's button2 handling) - each call reads the freshest
// ball data, updates the heading setpoint to face the ball, and drives
// forward at a slow, distance-limited speed.
//
// This does NOT reuse move()'s trapezoidal timing - move() is built for
// pre-planned segments with a known duration, while chasing the ball is
// open-ended and driven entirely by live camera feedback each tick, so
// it drives the motors directly via drive() instead, the same way move()
// does internally.
void chaseTick() {
  updateIMU();

  BallPacket ball;
  getLatestBallData(ball);

  unsigned long age_ms = millis() - lastBallPacketMs;
  bool haveFreshBall = ball.detected && (age_ms <= BALL_DATA_TIMEOUT_MS);

  if (!haveFreshBall) {
    // No recent, valid sighting - hold the last heading target rather
    // than guess, and don't translate.
    float rotation = headingCorrection();
    drive(0.0f, 0.0f, rotation);
    stopAllMotors();

#ifdef DEBUG_BALL_CHASE
    static unsigned long lastDebugMsA = 0;
    unsigned long nowMsA = millis();
    if (nowMsA - lastDebugMsA >= 100) {
      lastDebugMsA = nowMsA;
      Serial.print("chaseTick: no fresh ball data, ageMs=");
      Serial.println(age_ms);
    }
#endif
    return;
  }

  // Bearing to the ball in the CHASSIS frame (0 = straight ahead of the
  // chassis, +ve = ball to the right), after both rotation-offset
  // corrections (CAMERA_ROTATION_OFFSET_DEG already applied on the
  // OpenMV side, CAMERA_MOUNT_OFFSET_DEG applied here).
  float chassisRelativeBallAngle = ball.angleDeg + CAMERA_MOUNT_OFFSET_DEG;

  // Point the heading-hold setpoint at the ball. A chassis-relative
  // angle becomes a valid desiredHeadingDeg by adding the robot's
  // current (sign-corrected) yaw, so headingCorrection()'s PID is
  // always chasing a target expressed in the same world frame as
  // currentYawDeg - exactly the same trick move() uses via startYaw.
  float effectiveYaw = YAW_SIGN * currentYawDeg;
  desiredHeadingDeg = effectiveYaw + chassisRelativeBallAngle;

  float rotation = headingCorrection();

  // Drive toward BALL_TARGET_RADIUS_PX: farther away (bigger
  // radiusError) drives at BALL_CHASE_MAX_SPEED, then ramps down
  // linearly to BALL_CHASE_MIN_SPEED over the last
  // BALL_CHASE_RAMP_RANGE_PX px, and stops translating entirely once
  // radiusPx reaches the target (it keeps rotating to hold bearing on
  // the ball, it just stops driving forward). Combined with
  // desiredHeadingDeg above (which drives bearing toward 0 deg), the
  // goal state chaseTick() converges on is "ball at
  // BALL_TARGET_RADIUS_PX px, 0 deg bearing" - rotate to face the
  // ball, then close the distance, slowing down on approach.
  float radiusError = ball.radiusPx - BALL_TARGET_RADIUS_PX;
  float speed;

  if (radiusError <= 0.0f) {
    speed = 0.0f;
  } else {
    float t = constrain(radiusError / BALL_CHASE_RAMP_RANGE_PX, 0.0f, 1.0f);
    speed = BALL_CHASE_MIN_SPEED + t * (BALL_CHASE_MAX_SPEED - BALL_CHASE_MIN_SPEED);
  }

  // direction_deg passed to drive() must be FIELD-relative (drive()
  // re-derives the chassis-relative angle itself using currentYawDeg
  // every call - see drive()'s comments). We already have the ball's
  // angle in the chassis frame, so undo that conversion the same way
  // desiredHeadingDeg does above: add effectiveYaw so drive()'s internal
  // subtraction gets back to chassisRelativeBallAngle. This keeps the
  // robot strafing straight at the ball's last-known bearing even while
  // it's still rotating to face it.
  float fieldDirection = effectiveYaw + chassisRelativeBallAngle;

  drive(fieldDirection, speed, rotation);

#ifdef DEBUG_BALL_CHASE
  static unsigned long lastDebugMsB = 0;
  unsigned long nowMsB = millis();
  if (nowMsB - lastDebugMsB >= 100) {
    lastDebugMsB = nowMsB;
    Serial.print("bearing(chassis)=");
    Serial.print(chassisRelativeBallAngle);
    Serial.print(" radiusPx=");
    Serial.print(ball.radiusPx);
    Serial.print(" size=");
    Serial.print(ball.sizeByte);
    Serial.print(" speed=");
    Serial.print(speed);
    Serial.print(" ageMs=");
    Serial.println(age_ms);
  }
#endif
}
