#pragma once

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

// ============================================================
// Motor Pins
// ============================================================

constexpr int M1a = 2; // FRONT LEFT
constexpr int M1b = 3;
constexpr int M2a = 5; // FRONT RIGHT
constexpr int M2b = 4;
constexpr int M3a = 9; // BACK LEFT
constexpr int M3b = 8;
constexpr int M4a = 7; // BACK RIGHT
constexpr int M4b = 6;

// ============================================================
// Buttons
// ============================================================

constexpr int button1 = A6; // press: run the demo square sequence
constexpr int button2 = A7; // press: chase the ball for 20 seconds (see loop())
constexpr int button3 = A8;

// ============================================================
// OLED Display (SSD1306 over I2C)
// ============================================================

constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;
constexpr int OLED_RESET_PIN = -1;
constexpr uint8_t OLED_I2C_ADDRESS = 0x3C;

constexpr unsigned long DISPLAY_UPDATE_INTERVAL_MS = 250;

// ============================================================
// Ball Tracking (hardware UART link to OpenMV, Serial7)
// ============================================================

#define BALL_UART      Serial7
#define BALL_UART_BAUD 115200

constexpr uint8_t BALL_PACKET_LEN = 8;
constexpr uint8_t BALL_SYNC_A = 0xAA;
constexpr uint8_t BALL_SYNC_B = 0xAB;
constexpr uint8_t BALL_SYNC_C = 0xAC;

constexpr float CAMERA_MOUNT_OFFSET_DEG = 0.0f;
constexpr unsigned long BALL_DATA_TIMEOUT_MS = 300;
constexpr float BALL_CHASE_MAX_SPEED = 0.5f;
constexpr float BALL_CHASE_MIN_SPEED = 0.20f;
constexpr float BALL_TARGET_RADIUS_PX = 30.0f;
constexpr float BALL_CHASE_RAMP_RANGE_PX = 80.0f;

// ============================================================
// Battery Monitor
// ============================================================

constexpr int BATTERY_PIN = A2;
constexpr float BATTERY_DIVIDER_R1 = 4700.0f;
constexpr float BATTERY_DIVIDER_R2 = 1000.0f;
constexpr float BATTERY_DIVIDER_RATIO =
  (BATTERY_DIVIDER_R1 + BATTERY_DIVIDER_R2) / BATTERY_DIVIDER_R2;

constexpr int ADC_RESOLUTION_BITS = 12;
constexpr int ADC_MAX_VALUE = (1 << ADC_RESOLUTION_BITS) - 1;
constexpr float ADC_REF_VOLTAGE = 3.3f;

constexpr float BATTERY_SHUTDOWN_VOLTAGE = 14.8f;
constexpr unsigned long BATTERY_CHECK_INTERVAL_MS = 5000;
constexpr int BATTERY_SAMPLE_COUNT = 8;

// ============================================================
// Motor Calibration
// ============================================================

constexpr float motorMult[5] = {
  1.0f,
  1.00f,
  1.00f,
  1.00f,
  1.00f
};

// ============================================================
// Overall Speed Ceiling
// ============================================================

constexpr float ROBOT_MAX_SPEED = 0.5f;

// ============================================================
// Move / Acceleration Settings
// ============================================================

constexpr float ACCEL_LIMIT = 1.1f;
constexpr float ROTATION_ACCEL_LIMIT = 720.0f;
constexpr float ROTATION_MAX_SPEED = 240.0f;

// ============================================================
// Heading Hold Settings (PID)
// ============================================================

constexpr float HEADING_KP = 0.005f;
constexpr float HEADING_KI = 0.0f;
constexpr float HEADING_KD = 0.001f;
constexpr float HEADING_INTEGRAL_MAX = 0.20f;
constexpr float YAW_SIGN = 1.0f;
