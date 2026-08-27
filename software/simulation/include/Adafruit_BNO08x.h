#pragma once
// Minimal stub for Adafruit_BNO08x to satisfy includes during simulator builds.
using sh2_SensorValue_t = struct { union { struct { float real,i,j,k; } gameRotationVector; }; };
class Adafruit_BNO08x {
public:
  Adafruit_BNO08x(int reset) {}
  bool begin_I2C() { return false; }
  bool enableReport(int) { return false; }
  bool getSensorEvent(sh2_SensorValue_t*) { return false; }
  bool wasReset() { return false; }
};
