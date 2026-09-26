#pragma once
// Shared type definitions for the PlatformIO lidar pose test.

#include <Arduino.h>

struct MapEdge {
  float ax, ay, ex, ey;
};

struct RayHit {
  float range;
  bool vertical;
};

struct Ray {
  float dx, dy, range, weight;
  float invDx, invDy;  // 1/dx, 1/dy cached at append time -- fixed for the ray's lifetime
  unsigned long timestampMs;
};

struct FitResult {
  float cost, hxx, hxy, hyy, gx, gy, weightSum;
  uint16_t hitCount;
};

// distanceMm/intensity length is hardcoded to match POINTS_PER_PACKET (12)
// in lidar_pose_test.ino -- a static_assert there checks this stays in sync.
struct LidarPacket {
  uint16_t speed;
  uint16_t startAngle;
  uint16_t distanceMm[12];
  uint8_t intensity[12];
  uint16_t endAngle;
};

enum ReceiveState : uint8_t { WAIT_HEADER, WAIT_LENGTH, READ_PACKET };
