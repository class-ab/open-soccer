/*
  Merged robot sketch (Teensy 4.1)
  =================================
  Full robot control code (motors, IMU heading hold, OLED status display,
  battery protection, ball-chase) UNCHANGED, except the ball-tracking
  receiver has been swapped out.

  The previous receiver in this file bit-banged SCK/MOSI with no CS line
  and located packets by scanning a sliding 8-byte window for a sync byte
  + checksum. That approach did not work reliably.

  It has been replaced with the CS-FRAMED bit-banged receiver from the
  verified-working standalone test sketch ("teensy_dual_color_bitbang_slave"):
  CS going low marks the start of each 8-byte packet and CS going high
  marks its end, so the receiver always knows exactly where a packet
  starts/stops instead of having to search for one inside a continuous
  stream. This version also recognizes two sync bytes (0xAA / 0xAB, for
  two trackable colors upstream) - both are accepted here as "a ball"
  since this robot only chases one target.

  Everything else (motors, IMU, display, battery, move(), chaseTick(),
  drive()) is untouched from the original merged sketch.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Uncomment to print direction/yaw/vx/vy/wheel-speed diagnostics over
// Serial (about 10x/sec) during every move() - useful for confirming
// the YAW_SIGN convention and checking whether vx/vy are doing what you
// expect as the robot rotates.
// #define DEBUG_MOVE

// Uncomment to print ball-chase diagnostics (bearing, radius, size,
// computed speed, packet age) over Serial about 10x/sec while
// chaseTick() runs - useful for calibrating CAMERA_ROTATION_OFFSET_DEG
// (on the OpenMV side), CAMERA_MOUNT_OFFSET_DEG, BALL_TARGET_RADIUS_PX,
// and BALL_CHASE_RAMP_RANGE_PX.

#define DEBUG_BALL_CHASE

// Uncomment to print raw ball-link diagnostics (per-packet decode
// results and checksum failures) over Serial - useful for debugging the
// physical link itself (wiring, clock rate, sync/checksum issues),
// independent of the ball-chase behavior above. Off by default since it
// prints at packet rate, which is a lot busier than the throttled
// DEBUG_BALL_CHASE output.
// #define DEBUG_BALL_LINK

// ============================================================
// BNO08X IMU
// ============================================================

#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// ============================================================
// Motor Pins
// ============================================================

const int M1a = 2; // FRONT LEFT
const int M1b = 3;
const int M2a = 5; // FRONT RIGHT
const int M2b = 4;
const int M3a = 9; // BACK LEFT
const int M3b = 8;
const int M4a = 7; // BACK RIGHT
const int M4b = 6;

// ============================================================
// Buttons
// ============================================================

const int button1 = A6; // press: run the demo square sequence
const int button2 = A7; // press: chase the ball for 20 seconds (see loop())
const int button3 = A8;

// ============================================================
// OLED Display (SSD1306 over I2C)
// ============================================================

// Runs on Wire2, Teensy 4.1's native second-alternate I2C bus, which
// defaults to pins 24/25 - i.e. A10 (SDA2) / A11 (SCL2). This is a
// separate bus from the BNO08x (which stays on the default Wire /
// pins 18-19), so the two devices never contend for the bus.
//
// Change SCREEN_WIDTH/HEIGHT/OLED_I2C_ADDRESS if your module differs
// (0x3C is the common address for 128x64 and 128x32 SSD1306 boards;
// some 128x32 boards use 0x3D instead).
const int SCREEN_WIDTH = 128;
const int SCREEN_HEIGHT = 64;
const int OLED_RESET_PIN = -1; // most small SSD1306 boards have no reset pin
const uint8_t OLED_I2C_ADDRESS = 0x3C;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire2, OLED_RESET_PIN);

bool displayAvailable = false; // set true in initDisplay() if begin() succeeds

const unsigned long DISPLAY_UPDATE_INTERVAL_MS = 250;
unsigned long lastDisplayUpdateMs = 0;

// ============================================================
// Ball Tracking (bit-banged, CS-framed SPI-style link to OpenMV)
// ============================================================
//
// Replaces the previous no-CS / sliding-window receiver, which did not
// reliably lock onto packets. This version is ported directly from a
// standalone bit-banged CS-framed test receiver that was verified to
// work: the OpenMV cam drives SCK + MOSI (SPI mode 0, MSB-first) AND a
// CS line that goes low for the duration of each 8-byte packet and back
// high once it's done. CS framing means the receiver always knows
// exactly where a packet starts/ends instead of having to search for a
// sync byte inside a continuous, unframed stream - which is what made
// the old approach unreliable.
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
// Reception happens in two ISRs, ported unchanged (aside from naming)
// from the verified-working standalone test receiver:
//   - ballCSISR(), on CHANGE of BALL_SPI_CS_PIN: falling edge resets the
//     bit/byte counters (start of a new packet); rising edge means the
//     transfer is done, and if a full 8 bytes were clocked in, they're
//     copied into ballReadyBuf and ballPacketReady is set for the main
//     code to pick up.
//   - ballClockISR(), on RISING of BALL_SPI_SCK_PIN: bit-bangs one bit
//     of MOSI into the byte currently being assembled.
// processBallPacket() (called every tick from systemTick(), the same
// place decodeBallByteStream() used to be called from) drains
// ballPacketReady, validates the checksum + sync byte, and updates
// latestBallPacket / lastBallPacketMs - the same globals chaseTick()
// already reads via getLatestBallData().

const int BALL_SPI_SCK_PIN  = 13; // clock in from OpenMV
const int BALL_SPI_MOSI_PIN = 11; // data in from OpenMV
const int BALL_SPI_MISO_PIN = 12; // unused output, just kept configured
const int BALL_SPI_CS_PIN   = 10; // frames each 8-byte packet

const uint8_t BALL_PACKET_LEN = 8;
const uint8_t BALL_SYNC_A = 0xAA;
const uint8_t BALL_SYNC_B = 0xAB;

struct BallPacket {
  bool detected;
  float angleDeg;   // camera-frame bearing to the ball, see OpenMV script
  float radiusPx;   // pixel distance of the blob from the image center
  uint8_t sizeByte; // coarse blob-size indicator, 0-255
};

// Written from processBallPacket() (called regularly from systemTick(),
// defined near chaseTick() below), read from loop()/chaseTick() via
// getLatestBallData() - never read these two directly, the read isn't
// atomic across fields.
volatile BallPacket latestBallPacket = {false, 0.0f, 0.0f, 0};
volatile unsigned long lastBallPacketMs = 0;

// Raw reception state, filled by the two ISRs below. Ported directly
// from the standalone CS-framed test receiver that was used to verify
// this link works.
volatile uint8_t ballRxByte    = 0;
volatile uint8_t ballBitCount  = 0;
volatile uint8_t ballByteIndex = 0;
volatile uint8_t ballRxBuf[BALL_PACKET_LEN];

volatile uint8_t ballReadyBuf[BALL_PACKET_LEN];
volatile bool ballPacketReady = false;

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
const float CAMERA_MOUNT_OFFSET_DEG = 0.0f;

// If no valid packet arrives within this long, treat the ball as lost
// and stop translating rather than drive on stale data.
const unsigned long BALL_DATA_TIMEOUT_MS = 300;

// Speed ceiling for ball-chasing - keep this well below move()'s usual
// maxSpeed values. The whole point of chaseTick() is to approach the
// ball SLOWLY and under control, not to charge at it.
const float BALL_CHASE_MAX_SPEED = 0.3f;
const float BALL_CHASE_MIN_SPEED = 0.20f; // don't bother creeping below this

// Target radiusPx (packet bytes 4-5) the robot drives toward - the
// goal state for chaseTick() is "ball at BALL_TARGET_RADIUS_PX px, at
// 0 degrees bearing" (i.e. centered/aligned and this close). Once
// radiusPx reaches this value the robot stops translating (it keeps
// rotating to hold the bearing at the ball, it just stops driving
// forward). Tune this by watching the printed radiusPx value
// (DEBUG_BALL_CHASE) as you move the real ball closer to and farther
// from the camera.
const float BALL_TARGET_RADIUS_PX = 30.0f;

// How far past BALL_TARGET_RADIUS_PX (in px) the ball needs to be
// before the robot drives at the full BALL_CHASE_MAX_SPEED. Between
// BALL_TARGET_RADIUS_PX and BALL_TARGET_RADIUS_PX + BALL_CHASE_RAMP_RANGE_PX,
// speed scales down linearly from BALL_CHASE_MAX_SPEED to
// BALL_CHASE_MIN_SPEED, so the robot arrives at the target under
// control instead of stopping abruptly - farther away drives faster,
// closer in slows down. Tune alongside BALL_CHASE_MAX_SPEED/MIN_SPEED.
const float BALL_CHASE_RAMP_RANGE_PX = 80.0f;

// ============================================================
// Battery Monitor
// ============================================================

// Voltage divider: BATTERY+ --[4.7k]-- (A2 node) --[1k]-- GND
// Vnode = Vbatt * R2 / (R1 + R2)  =>  Vbatt = Vnode * (R1 + R2) / R2
const int BATTERY_PIN = A2;
const float BATTERY_DIVIDER_R1 = 4700.0f; // ohms, battery side
const float BATTERY_DIVIDER_R2 = 1000.0f; // ohms, ground side
const float BATTERY_DIVIDER_RATIO =
  (BATTERY_DIVIDER_R1 + BATTERY_DIVIDER_R2) / BATTERY_DIVIDER_R2; // 5.7

const int ADC_RESOLUTION_BITS = 12;
const int ADC_MAX_VALUE = (1 << ADC_RESOLUTION_BITS) - 1; // 4095
const float ADC_REF_VOLTAGE = 3.3f; // Teensy 4.x ADC reference

// At the shutdown threshold (14.7V), the divider node sits at
// 14.7 / 5.7 = 2.58V - comfortably under the 3.3V Teensy ADC max even
// well above the cutoff (e.g. a full 4S charge at ~16.8V -> ~2.95V).
const float BATTERY_SHUTDOWN_VOLTAGE = 14.7f;
const unsigned long BATTERY_CHECK_INTERVAL_MS = 5000;
const int BATTERY_SAMPLE_COUNT = 8; // oversampled per reading, for stability

unsigned long lastBatteryCheckMs = 0;
float lastBatteryVoltage = 0.0f;

// Latches true the moment a low-battery shutdown fires. Once set, the
// robot is intentionally dead until power-cycled - see
// emergencyShutdown() for why this isn't auto-resumable.
bool shutdownLatched = false;

// ============================================================
// Uptime / Run-Idle Timers
// ============================================================

unsigned long bootMillis = 0;       // millis() captured once in setup()
unsigned long lastRunStateChangeMs = 0; // millis() of the last run<->idle transition
bool robotCurrentlyRunning = false; // true for the duration of a move() sequence

// ============================================================
// Motor Calibration
// ============================================================

// 1.00 = nominal
// >1.00 = increase motor speed
// <1.00 = decrease motor speed

const float motorMult[5] = {
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

const float ACCEL_LIMIT = 1.1f;

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
int COUNT = 0;
const float ROTATION_ACCEL_LIMIT = 720.0f; // deg/s^2
const float ROTATION_MAX_SPEED = 240.0f;   // deg/s

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

const float HEADING_KP = 0.005f;
const float HEADING_KI = 0.0f;
const float HEADING_KD = 0.001f;

// Clamp on the accumulated integral term (anti-windup), expressed in the
// same units as the correction output (-1.0 .. 1.0 motor scale).
const float HEADING_INTEGRAL_MAX = 0.20f;

// Set to +1.0 or -1.0 depending on which way your IMU's yaw increases.
// TEST: hold the robot still and rotate it BY HAND counter-clockwise as
// seen from directly above (bird's-eye view). Watch currentYawDeg over
// Serial. If it INCREASES, leave this at +1.0. If it DECREASES, change
// this to -1.0. Getting this backwards is the single most common cause
// of a holonomic robot arcing instead of going straight while rotating.
const float YAW_SIGN = 1.0f;

float currentYawDeg = 0.0f;
float desiredHeadingDeg = 0.0f;

// PID internal state
float headingIntegral = 0.0f;
float headingLastError = 0.0f;
unsigned long headingLastTimeMs = 0;
bool headingPidInitialized = false;

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
// Function Prototypes
// ============================================================

void SetSpeed(int motor, int pwm);
void stopAllMotors();

void drive(
  float direction_deg,
  float speed,
  float rotation);

void move(
  float direction_deg,
  unsigned long duration_ms,
  float targetRotation_deg,
  float maxSpeed);

MoveProfile computeMoveProfile(
  float maxSpeed,
  float totalTime_sec,
  float accelLimit);

float speedAtTime(
  const MoveProfile &profile,
  float t_sec,
  float totalTime_sec,
  float accelLimit);

RotationProfile computeRotationProfile(
  float rotationDelta_deg,
  float accelLimit_degs2,
  float maxOmega_degs);

float angleAtTime(
  const RotationProfile &profile,
  float t_sec);

void setReports();
void updateIMU();

void initDisplay();
void updateDisplay();
float readBatteryVoltage();
void checkBattery();
void emergencyShutdown();
void systemTick();
String formatDuration(unsigned long ms);

void ballClockISR();
void ballCSISR();
void processBallPacket();
void getLatestBallData(BallPacket &out);
void chaseTick();

float quaternionToYawDegrees(
  float real,
  float i,
  float j,
  float k);

float angleError(
  float target,
  float current);

float headingCorrection();
void resetHeadingPID();

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(115200);

  pinMode(M1a, OUTPUT);
  pinMode(M1b, OUTPUT);

  pinMode(M2a, OUTPUT);
  pinMode(M2b, OUTPUT);

  pinMode(M3a, OUTPUT);
  pinMode(M3b, OUTPUT);

  pinMode(M4a, OUTPUT);
  pinMode(M4b, OUTPUT);

  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(button3, INPUT);

  delay(100);

  Serial.println("Starting...");

  bootMillis = millis();
  lastRunStateChangeMs = bootMillis;

  Wire2.begin();
  initDisplay();

  // Ball-tracking receiver (CS-framed, bit-banged). Safe to bring this
  // up before the OpenMV cam powers up - it just sits idle until CS/SCK
  // edges start arriving.
  pinMode(BALL_SPI_MOSI_PIN, INPUT);
  pinMode(BALL_SPI_SCK_PIN, INPUT);
  pinMode(BALL_SPI_CS_PIN, INPUT);
  pinMode(BALL_SPI_MISO_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(BALL_SPI_CS_PIN), ballCSISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BALL_SPI_SCK_PIN), ballClockISR, RISING);
  Serial.println("Ball-tracking CS-framed bit-banged receiver ready");

  analogReadResolution(ADC_RESOLUTION_BITS);

  // Take an immediate battery reading before anything else spins up, so
  // a dead/miswired/already-too-low battery is caught (and shuts things
  // down) before the BNO08x or motors are ever touched.
  lastBatteryCheckMs = millis();
  checkBattery();

  if (!bno08x.begin_I2C()) {
    Serial.println("BNO08x not found!");

    while (1) {
      delay(10);
    }
  }

  Serial.println("BNO08x Found");

  setReports();

  delay(500);

  updateIMU();

  Serial.print("Initial Heading: ");
  Serial.println(currentYawDeg);
}

// ============================================================
// Main Loop
// ============================================================

void loop() {
  systemTick(); // periodic battery check + display refresh (non-blocking)

  if (shutdownLatched) {
    // emergencyShutdown() already hangs forever internally, but this
    // guard is here too in case loop() is ever re-entered after a
    // shutdown for any reason - nothing below this point may run again.
    return;
  }

  updateIMU();

  if (digitalRead(button1) == HIGH) {
    delay(100);

    updateIMU();

    float startHeading = currentYawDeg;

    Serial.print("Starting Heading: ");
    Serial.println(startHeading);

    robotCurrentlyRunning = true;
    lastRunStateChangeMs = millis();

    // Example sequence: a 1.5s-per-side square, each side also gradually
    // rotating a quarter turn, unwinding back to the start heading on the
    // final leg.
    //
    // direction_deg is FIELD-relative (see drive()), so each side's
    // direction is offset by startHeading - this anchors "field forward"
    // to whichever way the robot happened to be facing at button press,
    // the same reference the rotation targets below use. The robot will
    // trace a straight-sided square in that fixed frame even though it's
    // spinning a quarter turn on every side.
    move(startHeading + 0,   1500, startHeading + 90.0f,  0.7f);
    move(startHeading + 90,  1500, startHeading + 180.0f, 0.7f);
    move(startHeading + 180, 1500, startHeading + 270.0f, 0.7f);
    move(startHeading + 270, 1500, startHeading,          0.7f);

    stopAllMotors();

    robotCurrentlyRunning = false;
    lastRunStateChangeMs = millis();
  }

  // Press button2 to chase the ball for BALL_CHASE_DURATION_MS (see
  // chaseTick() below), using the latest bearing/radius data from the
  // OpenMV cam. This is a single press, not press-and-hold - the chase
  // runs for a fixed window and then stops on its own. A dead battery
  // stops it immediately (via emergencyShutdown()); losing the ball for
  // longer than BALL_DATA_TIMEOUT_MS just makes chaseTick() hold heading
  // and stop translating until the ball is seen again or the window ends.
  chaseTick();

}

// ============================================================
// BNO08X Setup
// ============================================================

void setReports() {
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable rotation vector");
  }
}

// ============================================================
// Quaternion -> Yaw
// ============================================================

float quaternionToYawDegrees(
  float real,
  float i,
  float j,
  float k) {
  float yaw =
    atan2(
      2.0f * (real * k + i * j),
      1.0f - 2.0f * (j * j + k * k));

  return yaw * 180.0f / PI;
}

// ============================================================
// IMU Update
// ============================================================

void updateIMU() {
  if (bno08x.wasReset()) {
    setReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
    currentYawDeg =
      quaternionToYawDegrees(
        sensorValue.un.gameRotationVector.real,
        sensorValue.un.gameRotationVector.i,
        sensorValue.un.gameRotationVector.j,
        sensorValue.un.gameRotationVector.k);
  }
}

// ============================================================
// OLED Display
// ============================================================

void initDisplay() {
  displayAvailable = display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS);

  if (!displayAvailable) {
    // Not treated as fatal - battery protection and motion still work
    // fine without a display, this just means no on-robot readout.
    Serial.println("SSD1306 not found!");
    return;
  }

  display.clearDisplay();
  display.setRotation(2);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Booting...");
  display.display();
}

// Redraws the normal status screen: battery voltage, uptime since boot,
// and a run/idle timer (counts up while a move sequence is active,
// counts up from zero again once it stops - i.e. "how long has it been
// running" while running, "how long since it stopped" while idle).
void updateDisplay() {
  if (!displayAvailable || shutdownLatched) {
    // emergencyShutdown() owns the screen permanently once tripped.
    return;
  }

  unsigned long now = millis();
  unsigned long uptimeMs = now - bootMillis;
  unsigned long runStateMs = now - lastRunStateChangeMs;

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  display.print("Batt:   ");
  display.print(lastBatteryVoltage, 2);
  display.println("V");

  display.print("Uptime: ");
  display.println(formatDuration(uptimeMs));

  display.print("Status: ");
  display.println(robotCurrentlyRunning ? "RUNNING" : "STOPPED");

  display.print(robotCurrentlyRunning ? "Run tmr: " : "Idle tmr:");
  display.println(formatDuration(runStateMs));

  display.display();
}

// ============================================================
// Battery Monitor
// ============================================================

float readBatteryVoltage() {
  long sum = 0;

  for (int i = 0; i < BATTERY_SAMPLE_COUNT; i++) {
    sum += analogRead(BATTERY_PIN);
    delayMicroseconds(200);
  }

  float avgRaw = (float)sum / (float)BATTERY_SAMPLE_COUNT;
  float nodeVoltage = (avgRaw / (float)ADC_MAX_VALUE) * ADC_REF_VOLTAGE;

  return nodeVoltage * BATTERY_DIVIDER_RATIO;
}

// Reads the battery, stores it for the display, and immediately trips
// emergencyShutdown() if it's below BATTERY_SHUTDOWN_VOLTAGE. Called
// every BATTERY_CHECK_INTERVAL_MS by systemTick() - see that function
// for why it's also polled from inside move()'s loop, not just loop().
void checkBattery() {
  lastBatteryVoltage = readBatteryVoltage();
  lastBatteryCheckMs = millis();

  if (lastBatteryVoltage < BATTERY_SHUTDOWN_VOLTAGE) {
    emergencyShutdown();
  }
}

// Immediately halts the robot: stops every motor, latches the shutdown
// state, shows a permanent warning screen, and then hangs forever.
//
// This is intentionally NOT auto-resumable. A battery that has sagged
// below a safe under-voltage threshold usually keeps sagging further
// under load, and RCJ Soccer fields have no supervised "safe to
// continue" state to fall back to - the only reliable safe state is
// fully off, requiring a deliberate power cycle (with a charged/swapped
// battery) to clear.
void emergencyShutdown() {
  stopAllMotors();
  shutdownLatched = true;

  unsigned long uptimeMs = millis() - bootMillis;

  Serial.print("EMERGENCY SHUTDOWN - battery ");
  Serial.print(lastBatteryVoltage, 2);
  Serial.println("V");

  if (displayAvailable) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("!! LOW BATTERY !!");
    display.println("   SHUTDOWN");
    display.println("");
    display.print("Batt:  ");
    display.print(lastBatteryVoltage, 2);
    display.println("V");
    display.print("Limit: ");
    display.print(BATTERY_SHUTDOWN_VOLTAGE, 1);
    display.println("V");
    display.print("Uptime: ");
    display.println(formatDuration(uptimeMs));
    display.println("");
    display.println("Power cycle to reset");
    display.display();
  }

  while (true) {
    // Belt-and-braces: keep re-asserting motors off forever. Nothing
    // else in the program runs again after this point.
    stopAllMotors();
    delay(200);
  }
}

// Non-blocking scheduler for the two periodic background jobs: the 5s
// battery check (which can trip emergencyShutdown()) and the display
// refresh. Call this from loop() AND from inside move()'s while loop,
// so a battery cutoff is caught within one tick even mid-move rather
// than waiting for the current move to finish.
void systemTick() {
  unsigned long now = millis();

  processBallPacket(); // pick up any newly-completed ball packet

  if (now - lastBatteryCheckMs >= BATTERY_CHECK_INTERVAL_MS) {
    checkBattery(); // may call emergencyShutdown() and never return
  }

  if (now - lastDisplayUpdateMs >= DISPLAY_UPDATE_INTERVAL_MS) {
    lastDisplayUpdateMs = now;
    updateDisplay();
  }
}

// Formats a millisecond duration as "MM:SS", or "H:MM:SS" once it
// crosses an hour (uptime can run long across a full competition day).
String formatDuration(unsigned long ms) {
  unsigned long totalSeconds = ms / 1000UL;
  unsigned long hours = totalSeconds / 3600UL;
  unsigned long minutes = (totalSeconds % 3600UL) / 60UL;
  unsigned long seconds = totalSeconds % 60UL;

  char buf[16];

  if (hours > 0) {
    snprintf(buf, sizeof(buf), "%lu:%02lu:%02lu", hours, minutes, seconds);
  } else {
    snprintf(buf, sizeof(buf), "%02lu:%02lu", minutes, seconds);
  }

  return String(buf);
}

// ============================================================
// Heading Helpers
// ============================================================

float angleError(
  float target,
  float current) {
  float error = target - current;

  while (error > 180.0f)
    error -= 360.0f;

  while (error < -180.0f)
    error += 360.0f;

  return error;
}

// Clears all PID state (integral, last error, timer). Call this any time
// a fresh heading-hold segment starts (i.e. at the start of every move()
// or chaseTick() session), so old accumulated error from a previous
// segment doesn't bleed in and cause an overcorrection spike at the
// start of the next one.
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
MoveProfile computeMoveProfile(
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
float speedAtTime(
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
RotationProfile computeRotationProfile(
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
float angleAtTime(
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
// Ball Tracking Implementation
// ============================================================

// Fires on every CHANGE of BALL_SPI_CS_PIN. Falling edge = start of a
// new framed transfer (reset the bit/byte counters). Rising edge = the
// transfer is done; if a full 8 bytes were clocked in while CS was low,
// they're copied into ballReadyBuf and ballPacketReady is set for
// processBallPacket() to pick up. Ported directly from the standalone
// CS-framed test receiver that was used to verify this link works.
void ballCSISR() {
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
// stored into ballRxBuf. Ported directly from the standalone CS-framed
// test receiver, including its MSB-first shift-in. If bytes ever come
// out bit-reversed, change the shift below to LSB-first instead.
void ballClockISR() {
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

// Drains whatever ballCSISR() has queued up (ballPacketReady), validates
// the checksum and sync byte, and - if valid - updates latestBallPacket /
// lastBallPacketMs. Called every tick from systemTick(), so it runs
// regularly from loop() AND from inside move()'s/chaseTick()'s inner
// while-loops, the same way the battery check and display refresh do.
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
// latestBallPacket's fields directly from loop()/chaseTick(), since a
// multi-field struct read isn't guaranteed atomic with respect to the
// CS/clock ISRs running in between.
void getLatestBallData(BallPacket &out) {
  noInterrupts();
  out.detected = latestBallPacket.detected;
  out.angleDeg = latestBallPacket.angleDeg;
  out.radiusPx = latestBallPacket.radiusPx;
  out.sizeByte = latestBallPacket.sizeByte;
  interrupts();
}

// Runs one tick of "slowly drive towards the ball," using the latest
// packet from the OpenMV cam. Meant to be called repeatedly from a loop
// (see loop()'s button2 handling above) - each call reads the freshest
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
  // every call - see drive()'s comments above it). We already have the
  // ball's angle in the chassis frame, so undo that conversion the same
  // way desiredHeadingDeg does above: add effectiveYaw so drive()'s
  // internal subtraction gets back to chassisRelativeBallAngle. This
  // keeps the robot strafing straight at the ball's last-known bearing
  // even while it's still rotating to face it.
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
// Stop Motors
// ============================================================

void stopAllMotors() {
  SetSpeed(1, 0);
  SetSpeed(2, 0);
  SetSpeed(3, 0);
  SetSpeed(4, 0);
}

// ============================================================
// Motor Output
// ============================================================

void SetSpeed(int motor, int pwm) {
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
