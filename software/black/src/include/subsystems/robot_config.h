#pragma once

// Uncomment to print direction/yaw/vx/vy/wheel-speed diagnostics over
// Serial (about 10x/sec) during every move() - useful for confirming
// the YAW_SIGN convention and checking whether vx/vy are doing what you
// expect as the robot rotates.
// #define DEBUG_MOVE

// Uncomment to print raw ball-link diagnostics (per-packet decode
// results and checksum failures) over Serial - useful for debugging the
// physical link itself (wiring, clock rate, sync/checksum issues).
#define DEBUG_BALL_LINK
#define DEBUG_VISION_LINK
// #define DEBUG_BLUE_LINK
// #define DEBUG_YELLOW_LINK

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
// Dribbler ESC
// ============================================================

constexpr int DRIBBLER_THROTTLE_PIN = 23;
constexpr int DRIBBLER_REVERSE_PIN = 17;

constexpr int DRIBBLER_PULSE_MIN = 1000;
constexpr int DRIBBLER_PULSE_NEUTRAL = 1500;
constexpr int DRIBBLER_PULSE_MAX = 2000;

constexpr int DRIBBLER_FORWARD_US = 1000;
constexpr int DRIBBLER_REVERSE_US = 2000;

constexpr int DRIBBLER_RUN_THROTTLE_US = 1200;
constexpr unsigned long DRIBBLER_SPIN_TIME_MS = 3000;
constexpr unsigned long DRIBBLER_PAUSE_MS = 1000;

// ============================================================
// Buttons
// ============================================================

constexpr int button1 = A6;
constexpr int button2 = A7;
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
constexpr uint8_t BALL_SYNC = 0xAA;
constexpr uint8_t YELLOW_GOAL_SYNC = 0xAB;
constexpr uint8_t BLUE_GOAL_SYNC = 0xAC;


// ============================================================
// Overall Speed Ceiling
// ============================================================

constexpr float ROBOT_MAX_SPEED = 0.5f;
constexpr float ROTATION_MAX_SPEED = 0.5f;

constexpr float ROBOT_RAMP_RANGE = 20.0f;

constexpr unsigned long BALL_DATA_TIMEOUT_MS = 500;
constexpr float BALL_TARGET_DISTANCE_CM = 0.00f;

// "Has ball" is true when the ball is detected close enough and roughly
// in front of the robot (i.e. within the dribbler capture cone).
constexpr float HAS_BALL_MAX_DISTANCE_CM = 11.5f;
constexpr float HAS_BALL_MAX_ANGLE_DEG = 5.0f;

// ============================================================
// Localisation (goal-vector triangulation)
// ============================================================

// Target-follow profile constants (see localisation.h). Used to build a
// MoveProfile that drives holonomically toward a field point while rotating
// to a target heading.
constexpr float LOC_MAX_SPEED = ROBOT_MAX_SPEED;
constexpr float LOC_MIN_SPEED = 0.1f;
constexpr float LOC_RAMP_RANGE_CM = ROBOT_RAMP_RANGE;   // distance over which to ramp speed
constexpr float LOC_TARGET_REACH_DIST_CM = 0.0f; // stop once within this distance
// Heading error (deg) that produces full (1.0) rotation effort. Flip the sign
// if the robot rotates the wrong way in your heading convention.
constexpr float LOC_ROT_EFFORT_DEG = 45.0f;

// ============================================================
// Attack Strategy (ball chasing + goal scoring)
// ============================================================

// Attack phase-1 speed ramps toward the ball proportional to distance.
constexpr float CHASE_MIN_SPEED = 0.2f;
constexpr float CHASE_MAX_SPEED = ROBOT_MAX_SPEED;
constexpr float CHASE_RAMP_RANGE_CM = 30.0f;
// If the ball has not been detected for this long while attacking, fall back
// to returning home (used by the loop to dispatch to defend).
constexpr unsigned long BALL_LOST_RETURN_HOME_MS = 1000;

// Goal scoring: robot bearing R = G + O_G*M_G (front scoring, see article).
constexpr float GOAL_OFFSET_MULT = 0.0f;     // constant M_G while orbiting the goal
constexpr float SCORE_MIN_SPEED = 0.25f;
constexpr float SCORE_MAX_SPEED = ROBOT_MAX_SPEED;
constexpr float SCORE_RAMP_RANGE_CM = ROBOT_RAMP_RANGE;
// Below this distance from the goal we stop orbiting and drive straight into
// the goal with the ball (our "kicker" equivalent since we have no kicker).
constexpr float PUSH_DIST_CM = 60.0f;
constexpr float PUSH_SPEED = ROBOT_MAX_SPEED;

// When not attacking, the robot returns to a point this distance in front of
// its own goal (measured from the goal line toward the field centre).
constexpr float DEFEND_DIST_FROM_OWN_GOAL_CM = 30.0f;

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
constexpr unsigned long BATTERY_SHUTDOWN_DELAY_MS = 3000;
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
// Move / Acceleration Settings
// ============================================================

constexpr float ACCEL_LIMIT = 1.1f;
constexpr float ROTATION_ACCEL_LIMIT = 720.0f;

// ============================================================
// Heading Hold Settings (PID)
// ============================================================

constexpr float HEADING_KP = 0.005f;
constexpr float HEADING_KI = 0.0f;
constexpr float HEADING_KD = 0.0005f;
constexpr float HEADING_INTEGRAL_MAX = 0.01f;
constexpr float YAW_SIGN = 1.0f;
