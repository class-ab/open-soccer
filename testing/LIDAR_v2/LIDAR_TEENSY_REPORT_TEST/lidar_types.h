#pragma once
#include <Arduino.h>

// Split out into a header on purpose: the Arduino/Teensy IDE auto-generates forward
// declarations for every function in the .ino and inserts them right after the top
// #include block -- before it has parsed any struct defined further down in the same
// file. Keeping these types in a separate header means they're already visible (via
// the #include line) at the point those auto-generated prototypes get inserted.

struct LidarPacket {
  uint16_t speed;        // deg/s
  uint16_t startAngle;   // 0.01 deg units
  uint16_t distance_mm[12];
  uint8_t  intensity[12];
  uint16_t endAngle;     // 0.01 deg units
  uint16_t timestamp;    // ms, wraps at 30000
};

struct Pose { float x, y, cost; };