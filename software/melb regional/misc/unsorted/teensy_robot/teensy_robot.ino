#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Debug switches. Both are cheap (~10 prints/sec), leave on unless you
// need max loop speed.
#define DEBUG_MOVE          0  // vx/vy/wheel speed prints inside drive()
#define DEBUG_BALL_CHASE    1  // bearing/speed prints inside chaseTick()
#define DEBUG_BALL_LINK     1  // SPI packet/link health stats

// ============================================================
// BNO08X IMU
// ============================================================

#define BNO08X_RESET -1
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// ============================================================
// Motor Pins
// ============================================================

const int M1a = 2;  // FRONT LEFT
const int M1b = 3;
const int M2a = 5;  // FRONT RIGHT
const int M2b = 4;
const int M3a = 9;  // BACK LEFT
const int M3b = 8;
const int M4a = 7;  // BACK RIGHT
const int M4b = 6;

// ============================================================
// Buttons
// ============================================================

const int button1 = A6;  // press (edge): run the demo square sequence
const int button2 = A7;  // hold: chase the ball
const int button3 = A8;

// ============================================================
// OLED Display (SSD1306 over I2C, Wire2 / pins 24-25)
// ============================================================

const int SCREEN_WIDTH = 128;
const int SCREEN_HEIGHT = 64;
const int OLED_RESET_PIN = -1;
const uint8_t OLED_I2C_ADDRESS = 0x3C;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire2, OLED_RESET_PIN);
bool displayAvailable = false;

const unsigned long DISPLAY_UPDATE_INTERVAL_MS = 250;
unsigned long lastDisplayUpdateMs = 0;

// ============================================================
// Ball Tracking Link (Teensy = SPI0 HARDWARE SLAVE, bit-banged)
// ============================================================
//
// Wiring (Teensy SPI0 <-> OpenMV):
//   Teensy pin 10 (CS,   in) <- OpenMV P9  (SPI CS/NSS)
//   Teensy pin 11 (MOSI, in) <- OpenMV P0  (SPI MOSI)
//   Teensy pin 12 (MISO)     -- unused, not driven, left as input
//   Teensy pin 13 (SCK,  in) <- OpenMV P2  (SPI SCLK)
//   Teensy GND                -- OpenMV GND (required)
//
// Mode 0 (polarity=0, phase=0), MSB-first, matching the OpenMV script's
// `machine.SPI(1, baudrate=250000, polarity=0, phase=0)`. We don't use
// the Teensy's LPSPI peripheral in slave mode (no off-the-shelf slave
// driver in this build); instead the CS pin frames each transfer and an
// interrupt on SCK's rising edge (data valid edge for mode 0) shifts in
// each bit. CS framing is what makes this robust: every falling edge on
// CS resyncs the receiver to "start of packet" regardless of any
// previous glitch, and the byte-level sync+checksum scan below is kept
// as a second line of defense on top of that.
//
// Packet format (8 bytes, one per OpenMV frame) - matches the OpenMV
// script exactly, do not change without updating both sides:
//   byte 0:   sync byte, always 0xAA
//   byte 1:   detected flag, 0 or 1
//   byte 2-3: angle_deg * 100, signed 16-bit, big-endian
//   byte 4-5: radius_px, unsigned 16-bit, big-endian
//   byte 6:   coarse blob-size indicator, 0-255
//   byte 7:   checksum = XOR of bytes 0-6

const int BALL_SPI_CS_PIN = 10;    // CS in from OpenMV (active low)
const int BALL_SPI_MOSI_PIN = 11;  // data in from OpenMV
const int BALL_SPI_MISO_PIN = 12;  // unused, input only
const int BALL_SPI_SCK_PIN = 13;   // clock in from OpenMV

const uint8_t BALL_PACKET_SYNC_BYTE = 0xAA;
const uint8_t BALL_PACKET_LEN = 8;

struct BallPacket {
  bool detected;
  float angleDeg;
  float radiusPx;
  uint8_t sizeByte;
};

// Written by decodeBallByteStream() (via systemTick()), read via
// getLatestBallData(). Never read these two directly - the struct read
// isn't atomic against the ISRs.
volatile BallPacket latestBallPacket = { false, 0.0f, 0.0f, 0 };
volatile unsigned long lastBallPacketMs = 0;

// Raw-bit-stream reception state, ISR-owned except where noted.
const uint8_t BALL_RX_RING_SIZE = 64;  // 8 packets of headroom
volatile uint8_t ballSpiShiftReg = 0;
volatile uint8_t ballSpiBitCount = 0;
volatile uint8_t ballRxRing[BALL_RX_RING_SIZE];
volatile uint8_t ballRxRingHead = 0;  // next write index - ISR-owned
volatile uint8_t ballRxRingTail = 0;  // next read index - main-code-owned
volatile bool ballCsActive = false;

// Link health counters, for DEBUG_BALL_LINK. All ISR-incremented,
// main-code-read only (32-bit reads are not atomic on this core, so
// treat printed values as "approximately current", never used for
// control decisions).
volatile uint32_t ballStatBytesRx = 0;
volatile uint32_t ballStatPacketsOk = 0;
volatile uint32_t ballStatChecksumFail = 0;
volatile uint32_t ballStatResyncDrops = 0;
volatile uint32_t ballStatCsPartialByteDrops = 0;
unsigned long lastBallLinkDebugMs = 0;

// ---- Ball-chase tuning ----

// Added to the camera's already-corrected bearing to align it with the
// chassis's physical forward axis. Calibrate with the ball dead ahead
// until DEBUG_BALL_CHASE prints ~0 deg.
const float CAMERA_MOUNT_OFFSET_DEG = 0.0f;

const unsigned long BALL_DATA_TIMEOUT_MS = 300;
const float BALL_CHASE_MAX_SPEED = 0.25f;
const float BALL_CHASE_MIN_SPEED = 0.10f;
const uint8_t BALL_CLOSE_SIZE_BYTE = 40;  // stop translating above this

// ============================================================
// Battery Monitor
// ============================================================

// Divider: BATTERY+ --[4.7k]-- (A2 node) --[1k]-- GND
const int BATTERY_PIN = A2;
const float BATTERY_DIVIDER_R1 = 4700.0f;
const float BATTERY_DIVIDER_R2 = 1000.0f;
const float BATTERY_DIVIDER_RATIO =
  (BATTERY_DIVIDER_R1 + BATTERY_DIVIDER_R2) / BATTERY_DIVIDER_R2;  // 5.7

const int ADC_RESOLUTION_BITS = 12;
const int ADC_MAX_VALUE = (1 << ADC_RESOLUTION_BITS) - 1;
const float ADC_REF_VOLTAGE = 3.3f;

const float BATTERY_SHUTDOWN_VOLTAGE = 14.7f;
const unsigned long BATTERY_CHECK_INTERVAL_MS = 5000;
const int BATTERY_SAMPLE_COUNT = 8;

unsigned long lastBatteryCheckMs = 0;
float lastBatteryVoltage = 0.0f;
bool shutdownLatched = false;

// ============================================================
// Uptime / Run-Idle Timers
// ============================================================

unsigned long bootMillis = 0;
unsigned long lastRunStateChangeMs = 0;
bool robotCurrentlyRunning = false;

// ============================================================
// Motor Calibration
// ============================================================

const float motorMult[5] = { 1.0f, 1.00f, 1.00f, 1.00f, 1.00f };  // [0]=unused

// ============================================================
// Move / Acceleration Settings
// ============================================================

const float ACCEL_LIMIT = 1.1f;  // speed-fraction per second
const float ROTATION_ACCEL_LIMIT = 720.0f;  // deg/s^2
const float ROTATION_MAX_SPEED = 240.0f;    // deg/s

// ============================================================
// Heading Hold Settings (PID)
// ============================================================

// Tune order: KP, then KD, then KI (leave 0 unless drift never converges).
const float HEADING_KP = 0.005f;
const float HEADING_KI = 0.0f;
const float HEADING_KD = 0.001f;
const float HEADING_INTEGRAL_MAX = 0.20f;

// Rotate the robot BY HAND counter-clockwise (bird's-eye view) and watch
// currentYawDeg: increases -> leave at +1.0, decreases -> set -1.0.
const float YAW_SIGN = 1.0f;

float currentYawDeg = 0.0f;
float desiredHeadingDeg = 0.0f;

float headingIntegral = 0.0f;
float headingLastError = 0.0f;
unsigned long headingLastTimeMs = 0;
bool headingPidInitialized = false;

// ============================================================
// Move / Rotation Profiles
// ============================================================

struct MoveProfile {
  float accelTime;
  float cruiseTime;
  float decelTime;
  float peakSpeed;
};

// Solved for its own minimum completion time (independent of the
// enclosing move's duration_ms) - see computeRotationProfile().
struct RotationProfile {
  float accelTime;
  float cruiseTime;
  float decelTime;
  float peakOmegaMag;
  float effectiveAccelMag;
  float totalDelta;
  float rotationTime;
};

// ============================================================
// Function Prototypes
// ============================================================

void SetSpeed(int motor, int pwm);
void stopAllMotors();
void drive(float direction_deg, float speed, float rotation);
void move(float direction_deg, unsigned long duration_ms, float targetRotation_deg, float maxSpeed);
void runDemoSquare();

MoveProfile computeMoveProfile(float maxSpeed, float totalTime_sec, float accelLimit);
float speedAtTime(const MoveProfile &profile, float t_sec, float totalTime_sec, float accelLimit);
RotationProfile computeRotationProfile(float rotationDelta_deg, float accelLimit_degs2, float maxOmega_degs);
float angleAtTime(const RotationProfile &profile, float t_sec);

void setReports();
void updateIMU();

void initDisplay();
void updateDisplay();
float readBatteryVoltage();
void checkBattery();
void emergencyShutdown();
void systemTick();
String formatDuration(unsigned long ms);

void ballSpiSckISR();
void ballSpiCsISR();
void decodeBallByteStream();
void getLatestBallData(BallPacket &out);
void printBallLinkDebug();
void chaseTick();

float quaternionToYawDegrees(float real, float i, float j, float k);
float angleError(float target, float current);
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

  // Ball-tracking link. Safe to bring up before the OpenMV cam powers
  // on - it just idles until CS/clock edges arrive.
  pinMode(BALL_SPI_MOSI_PIN, INPUT);
  pinMode(BALL_SPI_MISO_PIN, INPUT);
  pinMode(BALL_SPI_SCK_PIN, INPUT);
  pinMode(BALL_SPI_CS_PIN, INPUT_PULLUP);  // idle high between packets
  attachInterrupt(digitalPinToInterrupt(BALL_SPI_CS_PIN), ballSpiCsISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BALL_SPI_SCK_PIN), ballSpiSckISR, RISING);
  Serial.println("Ball-tracking SPI0 link ready (CS=10 MOSI=11 SCK=13)");

  analogReadResolution(ADC_RESOLUTION_BITS);

  // Catch a dead/miswired/already-too-low battery before touching the
  // IMU or motors.
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
//
// Bug fixed here: the previous version called resetHeadingPID() and
// forced robotCurrentlyRunning = true on EVERY iteration, regardless of
// any button. That zeroed dt every tick (headingLastTimeMs kept getting
// reset to "now"), so the PID's I and D terms were permanently disabled
// during chase, chaseTick() ran unconditionally even with no button
// held, and the run/idle display timer never actually toggled. Fixed by
// only touching PID/state on an actual run<->idle transition, and by
// gating chaseTick() on button2.

bool prevButton2State = false;
bool prevButton1State = false;

void loop() {
  bool button2Held = digitalRead(button2) == HIGH;
  bool button1Held = digitalRead(button1) == HIGH;

  // Rising edge on button1 -> run the demo square once.
  if (button1Held && !prevButton1State) {
    runDemoSquare();
  }
  prevButton1State = button1Held;

  if (button2Held && !robotCurrentlyRunning) {
    robotCurrentlyRunning = true;
    lastRunStateChangeMs = millis();
    resetHeadingPID();  // fresh PID state for this chase session
  } else if (!button2Held && robotCurrentlyRunning) {
    robotCurrentlyRunning = false;
    lastRunStateChangeMs = millis();
    stopAllMotors();
  }
  prevButton2State = button2Held;

  if (robotCurrentlyRunning) {
    chaseTick();
  } else {
    systemTick();  // still service battery/display/ball-link while idle
  }

  delay(1);
}

// ============================================================
// BNO08X Setup
// ============================================================

void setReports() {
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable rotation vector");
  }
}

float quaternionToYawDegrees(float real, float i, float j, float k) {
  float yaw = atan2(2.0f * (real * k + i * j), 1.0f - 2.0f * (j * j + k * k));
  return yaw * 180.0f / PI;
}

void updateIMU() {
  if (bno08x.wasReset()) {
    setReports();
  }
  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }
  if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
    currentYawDeg = quaternionToYawDegrees(
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
    Serial.println("SSD1306 not found!");  // not fatal, rest still works
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

void updateDisplay() {
  if (!displayAvailable || shutdownLatched) {
    return;  // emergencyShutdown() owns the screen permanently once tripped
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

  bool ballFresh = (millis() - lastBallPacketMs) <= BALL_DATA_TIMEOUT_MS;
  display.print("Ball:   ");
  display.println(ballFresh ? "TRACKED" : "lost");

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

void checkBattery() {
  lastBatteryVoltage = readBatteryVoltage();
  lastBatteryCheckMs = millis();
  if (lastBatteryVoltage < BATTERY_SHUTDOWN_VOLTAGE) {
    emergencyShutdown();
  }
}

// Not auto-resumable: a sagged battery keeps sagging under load, and
// there's no safe "continue" state - only a full power cycle clears it.
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
    stopAllMotors();
    delay(200);
  }
}

// Non-blocking scheduler for the periodic background jobs. Call from
// loop() AND from inside move()'s/runDemoSquare()'s loops so a battery
// cutoff and ball-packet decode happen within one tick even mid-move.
void systemTick() {
  unsigned long now = millis();

  decodeBallByteStream();

  if (now - lastBatteryCheckMs >= BATTERY_CHECK_INTERVAL_MS) {
    checkBattery();  // may call emergencyShutdown() and never return
  }

  if (now - lastDisplayUpdateMs >= DISPLAY_UPDATE_INTERVAL_MS) {
    lastDisplayUpdateMs = now;
    updateDisplay();
  }

#if DEBUG_BALL_LINK
  if (now - lastBallLinkDebugMs >= 1000) {
    lastBallLinkDebugMs = now;
    printBallLinkDebug();
  }
#endif
}

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

float angleError(float target, float current) {
  float error = target - current;
  while (error > 180.0f) error -= 360.0f;
  while (error < -180.0f) error += 360.0f;
  return error;
}

void resetHeadingPID() {
  headingIntegral = 0.0f;
  headingLastError = 0.0f;
  headingLastTimeMs = millis();
  headingPidInitialized = false;
}

float headingCorrection() {
  unsigned long now = millis();
  float error = angleError(desiredHeadingDeg, currentYawDeg);
  float dt = 0.0f;

  if (headingPidInitialized) {
    dt = (now - headingLastTimeMs) / 1000.0f;
  }

  if (dt > 0.0f) {
    headingIntegral += error * dt;
    headingIntegral = constrain(headingIntegral, -HEADING_INTEGRAL_MAX, HEADING_INTEGRAL_MAX);
  }

  float derivative = 0.0f;
  if (dt > 0.0f) {
    derivative = (error - headingLastError) / dt;
  }

  float correction = (error * HEADING_KP) + (headingIntegral * HEADING_KI) + (derivative * HEADING_KD);
  correction = constrain(correction, -0.40f, 0.40f);

  headingLastError = error;
  headingLastTimeMs = now;
  headingPidInitialized = true;

  return correction;
}

// ============================================================
// Move Profile: trapezoidal speed ramp solver
// ============================================================

MoveProfile computeMoveProfile(float maxSpeed, float totalTime_sec, float accelLimit) {
  MoveProfile profile;
  maxSpeed = constrain(maxSpeed, 0.0f, 1.0f);

  if (accelLimit <= 0.0001f || totalTime_sec <= 0.0001f) {
    profile.accelTime = 0.0f;
    profile.decelTime = 0.0f;
    profile.cruiseTime = totalTime_sec;
    profile.peakSpeed = maxSpeed;
    return profile;
  }

  float accelTimeFull = maxSpeed / accelLimit;

  if ((accelTimeFull * 2.0f) <= totalTime_sec) {
    profile.accelTime = accelTimeFull;
    profile.decelTime = accelTimeFull;
    profile.cruiseTime = totalTime_sec - (accelTimeFull * 2.0f);
    profile.peakSpeed = maxSpeed;
  } else {
    profile.accelTime = totalTime_sec / 2.0f;
    profile.decelTime = totalTime_sec / 2.0f;
    profile.cruiseTime = 0.0f;
    profile.peakSpeed = accelLimit * profile.accelTime;
  }

  return profile;
}

float speedAtTime(const MoveProfile &profile, float t_sec, float totalTime_sec, float accelLimit) {
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
// Rotation Profile: trapezoidal angle ramp solver (own min-time,
// independent of the enclosing move's duration_ms)
// ============================================================

RotationProfile computeRotationProfile(float rotationDelta_deg, float accelLimit_degs2, float maxOmega_degs) {
  RotationProfile profile;
  profile.totalDelta = rotationDelta_deg;

  float absDelta = fabs(rotationDelta_deg);
  float maxOmega = fabs(maxOmega_degs);

  if (absDelta <= 0.0001f || accelLimit_degs2 <= 0.0001f || maxOmega <= 0.0001f) {
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
    float t_a = sqrt(absDelta / a);
    profile.accelTime = t_a;
    profile.decelTime = t_a;
    profile.cruiseTime = 0.0f;
    profile.peakOmegaMag = a * t_a;
    profile.effectiveAccelMag = a;
    profile.rotationTime = 2.0f * t_a;
  } else {
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

float angleAtTime(const RotationProfile &profile, float t_sec) {
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
      float magAtCruiseEnd = magAtAccelEnd + (profile.peakOmegaMag * profile.cruiseTime);
      float tIntoDecel = t_sec - cruiseEnd;
      mag = magAtCruiseEnd + (profile.peakOmegaMag * tIntoDecel) - (0.5f * a * tIntoDecel * tIntoDecel);
    }
  }

  mag = constrain(mag, 0.0f, absDelta);
  return sign * mag;
}

// ============================================================
// Move: vector direction + accel-limited speed + independently
//       accel-limited, as-fast-as-possible rotation
// ============================================================

void move(float direction_deg, unsigned long duration_ms, float targetRotation_deg, float maxSpeed) {
  if (duration_ms == 0) {
    stopAllMotors();
    return;
  }

  updateIMU();
  float startYaw = currentYawDeg;
  float rotationDelta = angleError(targetRotation_deg, startYaw);
  float totalTime_sec = duration_ms / 1000.0f;

  MoveProfile profile = computeMoveProfile(maxSpeed, totalTime_sec, ACCEL_LIMIT);
  RotationProfile rotationProfile = computeRotationProfile(rotationDelta, ROTATION_ACCEL_LIMIT, ROTATION_MAX_SPEED);

  resetHeadingPID();
  unsigned long moveStart = millis();

  while (true) {
    unsigned long elapsed_ms = millis() - moveStart;
    if (elapsed_ms >= duration_ms) {
      break;
    }

    systemTick();  // battery/ball-link/display service mid-move
    updateIMU();

    float t_sec = elapsed_ms / 1000.0f;
    float speed = speedAtTime(profile, t_sec, totalTime_sec, ACCEL_LIMIT);
    desiredHeadingDeg = startYaw + angleAtTime(rotationProfile, t_sec);
    float rotation = headingCorrection();

    drive(direction_deg, speed, rotation);
    delay(5);
  }

  // Left wherever the rotation profile actually got to (no forced snap) -
  // the next move() reads true current yaw, so nothing desyncs.
  stopAllMotors();
}

// Simple 4-side square using move(), triggered by button1's rising edge.
void runDemoSquare() {
  robotCurrentlyRunning = true;
  lastRunStateChangeMs = millis();

  updateIMU();
  float startYaw = currentYawDeg;

  for (int side = 0; side < 4; side++) {
    move(startYaw + (side * 90.0f), 1000, startYaw, 0.5f);
  }

  robotCurrentlyRunning = false;
  lastRunStateChangeMs = millis();
}

// ============================================================
// Ball Tracking Link
// ============================================================

// CS from OpenMV, active low. FALLING = start of an 8-byte transfer:
// resync the bit-shifter so this packet always starts clean regardless
// of anything before it. RISING = end of transfer: if a byte was left
// mid-shift, that's noise/a dropped bit, so discard it rather than let
// it bleed into the next packet.
void ballSpiCsISR() {
  bool csLow = digitalReadFast(BALL_SPI_CS_PIN) == LOW;

  if (csLow) {
    ballCsActive = true;
    ballSpiShiftReg = 0;
    ballSpiBitCount = 0;
  } else {
    if (ballCsActive && ballSpiBitCount != 0) {
      ballStatCsPartialByteDrops++;
    }
    ballCsActive = false;
    ballSpiShiftReg = 0;
    ballSpiBitCount = 0;
  }
}

// Rising edge on SCK = data-valid edge for SPI mode 0. Shifts one bit
// per edge; pushes a completed byte into the ring buffer.
void ballSpiSckISR() {
  if (!ballCsActive) {
    return;  // ignore clock noise while CS is deasserted
  }

  uint8_t bit = digitalReadFast(BALL_SPI_MOSI_PIN);
  ballSpiShiftReg = (uint8_t)((ballSpiShiftReg << 1) | bit);
  ballSpiBitCount++;

  if (ballSpiBitCount >= 8) {
    ballStatBytesRx++;
    uint8_t nextHead = (uint8_t)((ballRxRingHead + 1) % BALL_RX_RING_SIZE);

    if (nextHead != ballRxRingTail) {
      ballRxRing[ballRxRingHead] = ballSpiShiftReg;
      ballRxRingHead = nextHead;
    }
    // Ring full: drop this byte rather than overwrite unread data -
    // decodeBallByteStream() resyncs on the next valid packet regardless.

    ballSpiShiftReg = 0;
    ballSpiBitCount = 0;
  }
}

// Scans the raw byte ring for a full, checksum-valid packet, updating
// latestBallPacket when found. Repeats until fewer than
// BALL_PACKET_LEN bytes remain. Called every tick from systemTick().
void decodeBallByteStream() {
  while (true) {
    uint8_t available = (uint8_t)((ballRxRingHead - ballRxRingTail + BALL_RX_RING_SIZE) % BALL_RX_RING_SIZE);

    if (available < BALL_PACKET_LEN) {
      return;
    }

    uint8_t firstByte = ballRxRing[ballRxRingTail];

    if (firstByte != BALL_PACKET_SYNC_BYTE) {
      ballRxRingTail = (uint8_t)((ballRxRingTail + 1) % BALL_RX_RING_SIZE);
      ballStatResyncDrops++;
      continue;
    }

    uint8_t buf[BALL_PACKET_LEN];
    for (uint8_t i = 0; i < BALL_PACKET_LEN; i++) {
      buf[i] = ballRxRing[(ballRxRingTail + i) % BALL_RX_RING_SIZE];
    }

    uint8_t checksum = 0;
    for (uint8_t i = 0; i < BALL_PACKET_LEN - 1; i++) {
      checksum ^= buf[i];
    }

    if (checksum != buf[BALL_PACKET_LEN - 1]) {
      ballRxRingTail = (uint8_t)((ballRxRingTail + 1) % BALL_RX_RING_SIZE);
      ballStatChecksumFail++;
      continue;
    }

    ballRxRingTail = (uint8_t)((ballRxRingTail + BALL_PACKET_LEN) % BALL_RX_RING_SIZE);

    int16_t angle_x100 = (int16_t)((buf[2] << 8) | buf[3]);
    uint16_t radius_i = (uint16_t)((buf[4] << 8) | buf[5]);

    latestBallPacket.detected = buf[1] != 0;
    latestBallPacket.angleDeg = angle_x100 / 100.0f;
    latestBallPacket.radiusPx = (float)radius_i;
    latestBallPacket.sizeByte = buf[6];
    lastBallPacketMs = millis();
    ballStatPacketsOk++;
  }
}

void getLatestBallData(BallPacket &out) {
  noInterrupts();
  out.detected = latestBallPacket.detected;
  out.angleDeg = latestBallPacket.angleDeg;
  out.radiusPx = latestBallPacket.radiusPx;
  out.sizeByte = latestBallPacket.sizeByte;
  interrupts();
}

void printBallLinkDebug() {
  unsigned long age_ms = millis() - lastBallPacketMs;
  Serial.print("[ballLink] bytes=");
  Serial.print(ballStatBytesRx);
  Serial.print(" pktsOk=");
  Serial.print(ballStatPacketsOk);
  Serial.print(" chkFail=");
  Serial.print(ballStatChecksumFail);
  Serial.print(" resync=");
  Serial.print(ballStatResyncDrops);
  Serial.print(" csPartial=");
  Serial.print(ballStatCsPartialByteDrops);
  Serial.print(" lastPktAgeMs=");
  Serial.println(age_ms);
}

// Slowly drives towards the ball using the latest OpenMV packet. Driven
// entirely by live feedback each call (not move()'s pre-planned timing),
// so it calls drive() directly.
void chaseTick() {
  updateIMU();

  BallPacket ball;
  getLatestBallData(ball);

  unsigned long age_ms = millis() - lastBallPacketMs;
  bool haveFreshBall = ball.detected && (age_ms <= BALL_DATA_TIMEOUT_MS);

  systemTick();

  if (!haveFreshBall) {
    float rotation = headingCorrection();
    drive(0.0f, 0.0f, rotation);

#if DEBUG_BALL_CHASE
    static unsigned long lastDebugMsA = 0;
    unsigned long nowMsA = millis();
    if (nowMsA - lastDebugMsA >= 100) {
      lastDebugMsA = nowMsA;
      Serial.print("chaseTick: no fresh ball, ageMs=");
      Serial.println(age_ms);
    }
#endif
    return;
  }

  // Chassis-relative bearing (0 = ahead, +ve = right), after the
  // chassis-mount offset correction on top of the OpenMV-side one.
  float chassisRelativeBallAngle = ball.angleDeg + CAMERA_MOUNT_OFFSET_DEG;

  float effectiveYaw = YAW_SIGN * currentYawDeg;
  desiredHeadingDeg = effectiveYaw + chassisRelativeBallAngle;
  float rotation = headingCorrection();

  float speed;
  if (ball.sizeByte >= BALL_CLOSE_SIZE_BYTE) {
    speed = 0.0f;
  } else {
    speed = BALL_CHASE_MAX_SPEED;
    if (speed < BALL_CHASE_MIN_SPEED) {
      speed = BALL_CHASE_MIN_SPEED;
    }
  }

  // drive() re-derives chassis-relative angle from field-relative input
  // using currentYawDeg, so undo the frame conversion the same way
  // desiredHeadingDeg does above.
  float fieldDirection = effectiveYaw + chassisRelativeBallAngle;
  drive(fieldDirection, speed, rotation);

#if DEBUG_BALL_CHASE
  static unsigned long lastDebugMsB = 0;
  unsigned long nowMsB = millis();
  if (nowMsB - lastDebugMsB >= 100) {
    lastDebugMsB = nowMsB;
    Serial.print("bearing=");
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

// direction_deg is FIELD-relative (fixed world angle, not chassis-
// relative). Re-projected into the chassis's current body frame every
// call using currentYawDeg, so translation stays fixed in the world
// even while the chassis rotates underneath it - the standard
// field-oriented holonomic transform.
void drive(float direction_deg, float speed, float rotation) {
  speed = constrain(speed, 0.0f, 1.0f);

  float effectiveYaw = YAW_SIGN * currentYawDeg;
  float body_direction_rad = (direction_deg - effectiveYaw) * PI / 180.0f;

  float vx = speed * cos(body_direction_rad);
  float vy = speed * sin(body_direction_rad);

  float wheel_speeds[4];

  // v = -vx*sin(theta) + vy*cos(theta) + rotation, theta = wheel mount
  // angle from chassis forward.
  wheel_speeds[0] = -vx * sin(45 * PI / 180.0f) + vy * cos(45 * PI / 180.0f) + rotation;
  wheel_speeds[1] = -vx * sin(-45 * PI / 180.0f) + vy * cos(-45 * PI / 180.0f) + rotation;
  wheel_speeds[2] = -vx * sin(-135 * PI / 180.0f) + vy * cos(-135 * PI / 180.0f) + rotation;
  wheel_speeds[3] = -vx * sin(135 * PI / 180.0f) + vy * cos(135 * PI / 180.0f) + rotation;

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

#if DEBUG_MOVE
  static unsigned long lastDebugMs = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastDebugMs >= 100) {
    lastDebugMs = nowMs;
    Serial.print("dir="); Serial.print(direction_deg);
    Serial.print(" yaw="); Serial.print(currentYawDeg);
    Serial.print(" bodyDir="); Serial.print(body_direction_rad * 180.0f / PI);
    Serial.print(" vx="); Serial.print(vx);
    Serial.print(" vy="); Serial.print(vy);
    Serial.print(" rot="); Serial.print(rotation);
    Serial.print(" w=["); Serial.print(wheel_speeds[0]);
    Serial.print(","); Serial.print(wheel_speeds[1]);
    Serial.print(","); Serial.print(wheel_speeds[2]);
    Serial.print(","); Serial.print(wheel_speeds[3]);
    Serial.println("]");
  }
#endif

  SetSpeed(1, wheel_speeds[0] * 255);
  SetSpeed(2, wheel_speeds[1] * 255);
  SetSpeed(3, wheel_speeds[3] * 255);
  SetSpeed(4, wheel_speeds[2] * 255);
}

void stopAllMotors() {
  SetSpeed(1, 0);
  SetSpeed(2, 0);
  SetSpeed(3, 0);
  SetSpeed(4, 0);
}

void SetSpeed(int motor, int pwm) {
  pwm = (int)(pwm * motorMult[motor]);
  pwm = constrain(pwm, -255, 255);

  int pinA;
  int pinB;

  switch (motor) {
    case 1: pinA = M1a; pinB = M1b; break;
    case 2: pinA = M2a; pinB = M2b; break;
    case 3: pinA = M3a; pinB = M3b; break;
    case 4: pinA = M4a; pinB = M4b; break;
    default: return;
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
