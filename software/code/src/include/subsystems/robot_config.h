#pragma once

// On real hardware (ARDUINO) each binary controls exactly one robot, so these
// expand to nothing. The simulator runs multiple robot instances as separate
// threads and defines ARDUINO nowhere, so SIM_TLS/SIM_STATIC_TLS give each
// thread its own isolated copy of otherwise-shared firmware state.
#if defined(ARDUINO)
#define SIM_TLS
#define SIM_STATIC_TLS static
#else
#define SIM_TLS thread_local
#define SIM_STATIC_TLS thread_local
#endif

// SPEED LIMITS
constexpr float ROBOT_MAX_SPEED = 1.0f;
constexpr float ROTATION_MAX_SPEED = 0.5f;
// Calibrate against measured full-command robot speed on the actual field.
constexpr float ROBOT_LINEAR_SPEED_MM_S = 1800.0f;

// ACCELERATION LIMITS
constexpr float ACCEL_LIMIT = 1.0f;
constexpr float ROTATION_ACCEL_LIMIT = 1.0f;

// moveTo() position-to-speed proportional gain (normalized speed / mm).
// Gentle preset (see set list below) -- previous value was ~4x weaker than
// even that floor and produced too little PWM to overcome motor deadband.
constexpr float POSITION_KP = 0.00065f;
constexpr float POSITION_TOLERANCE_MM = 10.0f;
constexpr float HEADING_TOLERANCE_DEG = 1.0f;

// Heading PID output is normalized rotation command. D uses IMU yaw rate.
constexpr float HEADING_KP = 0.005f;
constexpr float HEADING_KI = 0.0002f;
constexpr float HEADING_KD = 0.001f;
// Integral is accumulated in degree-seconds and clamped before applying KI.
constexpr float HEADING_INTEGRAL_MAX = 100.0f;
constexpr float YAW_SIGN = 1.0f;

// Example starting sets for controlled tuning (select one set at a time):
// Gentle:     POSITION_KP 0.0007, HEADING_KP 0.0030, KI 0.0002, KD 0.0012
// Balanced:   POSITION_KP 0.0012, HEADING_KP 0.0050, KI 0.0000, KD 0.0025
// Responsive: POSITION_KP 0.0018, HEADING_KP 0.0070, KI 0.0005, KD 0.0015

// ball sensing
constexpr float CAMERA_MOUNT_OFFSET_DEG = 0.0f;
constexpr unsigned long BALL_DATA_TIMEOUT_MS = 300;
constexpr float BALL_TARGET_DISTANCE_CM = 20.00f;

// #define DEBUG_MOVE
// #define DEBUG_BALL_LINK
// #define DEBUG_LIDAR        // basic lidar info
// #define DEBUG_FIELDBALL    // requires DEBUG_LIDAR
// #define LOCAL_STATE
// #define LIDAR_POSE_STREAM  // advanced lidar debug + py script

// IMU
#define BNO08X_RESET -1

// motor pins
constexpr int M1a = 2; // FRONT LEFT
constexpr int M1b = 3;
constexpr int M2a = 5; // FRONT RIGHT
constexpr int M2b = 4;
constexpr int M3a = 9; // BACK LEFT
constexpr int M3b = 8;
constexpr int M4a = 7; // BACK RIGHT
constexpr int M4b = 6;

// dribbler ESC
constexpr int DRIBBLER_THROTTLE_PIN = 23;
constexpr int DRIBBLER_REVERSE_PIN = 17;
constexpr int DRIBBLER_PULSE_MIN = 1000;
constexpr int DRIBBLER_PULSE_NEUTRAL = 1500;
constexpr int DRIBBLER_PULSE_MAX = 2000;
constexpr int DRIBBLER_FORWARD_US = 1000;
constexpr int DRIBBLER_REVERSE_US = 2000;
constexpr int DRIBBLER_RUN_THROTTLE_US = 1200; // MAIN SPEED CHANGE THIS

// kicker pins
constexpr int kicker = 32;
constexpr int charge = 31;

// buttons
constexpr int button1 = A7;
constexpr int button2 = A6;
constexpr int button3 = A8;
constexpr unsigned long BUTTON_DEBOUNCE_MS = 1;

// OLED
constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;
constexpr int OLED_RESET_PIN = -1;
constexpr uint8_t OLED_I2C_ADDRESS = 0x3C;
constexpr unsigned long DISPLAY_UPDATE_INTERVAL_MS = 80;

// OpenMV 
#define BALL_UART      Serial7
#define BALL_UART_BAUD 115200
constexpr uint8_t BALL_PACKET_LEN = 8;
constexpr uint8_t BALL_SYNC = 0xAA;
constexpr uint8_t YELLOW_GOAL_SYNC = 0xAB; // not used currently
constexpr uint8_t BLUE_GOAL_SYNC = 0xAC; // not used currently

// LD14P LiDAR
#define LIDAR_UART      Serial3
#define LIDAR_UART_BAUD 230400
// LD14P motor speed PWM control loop (closed-loop on the reported scan speed).
constexpr uint8_t LIDAR_SPEED_CONTROL_PIN = 14;
constexpr uint32_t LIDAR_PWM_FREQUENCY_HZ = 1000;
// Measured working duty for this robot's LD14P is ~80% -- start near there so
// boot doesn't have to slowly ramp up from a much lower entry point.
constexpr float LIDAR_PWM_ENTRY_DUTY_PERCENT = 78.0f;
constexpr float LIDAR_PWM_MIN_DUTY_PERCENT = 45.1f;
constexpr float LIDAR_PWM_MAX_DUTY_PERCENT = 80.0f;
constexpr float LIDAR_TARGET_SPEED_DEG_S = 2880.0f;
constexpr float LIDAR_SPEED_TOLERANCE_DEG_S = 36.0f;
constexpr unsigned long LIDAR_CONTROL_INTERVAL_MS = 500;

// battery
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
constexpr int BATTERY_SAMPLE_COUNT = 3;

// motor calibration (currently unused)
constexpr float motorMult[5] = {
  1.0f,
  1.00f,
  1.00f,
  1.00f,
  1.00f
};
