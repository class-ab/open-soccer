#pragma once

#include <cstdint>
#include <string>

using byte = uint8_t;
using uint8 = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using String = std::string;

// basic Arduino constants
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1

// Analog pin aliases commonly used in the code (A0..A15)
#ifndef A0
#define A0 0
#define A1 1
#define A2 2
#define A3 3
#define A4 4
#define A5 5
#define A6 6
#define A7 7
#define A8 8
#define A9 9
#define A10 10
#define A11 11
#define A12 12
#define A13 13
#define A14 14
#define A15 15
#endif

// simple typedefs
using unsigned_long = unsigned long;

// minimal Serial stub forward declaration
class HardwareSerial {
public:
  void begin(unsigned long);
  void print(const std::string &s);
  void print(float v, int precision = 2);
  void println(const std::string &s);
  void println(float v);
  void println(int v);
  void println(unsigned long v);
  void println();
  int available();
  int read();
  void write(uint8_t);
};

extern HardwareSerial Serial;
extern HardwareSerial Serial7; // used as BALL_UART

// math constants
#ifndef PI
constexpr float PI = 3.14159265358979323846f;
#endif

// Wire stub
class TwoWire {
public:
  void begin() {}
};
extern TwoWire Wire2;

// Arduino API
extern unsigned long millis();
extern void delay(unsigned long ms);
extern void delayMicroseconds(unsigned int us);

// Simulator helpers: allow the simulator to drive the millis() epoch and enable/disable the robot thread.
// These are implemented in sim_hal.cpp and are only meaningful when building/running the simulator.
extern void sim_set_millis(unsigned long ms);
// Note: sim_set_robot_enabled exists for backwards compatibility but main now uses
// robotCurrentlyRunning to control firmware state. The function is kept in sim_hal
// but should not be required for normal operation.
extern void sim_set_robot_enabled(bool enabled);

extern void pinMode(int pin, int mode);
extern int digitalRead(int pin);
extern void digitalWrite(int pin, int value);
extern int analogRead(int pin);
extern void analogWrite(int pin, int value);
extern void analogReadResolution(int bits);

// Simple Servo API stubs
class Servo {
public:
  void attach(int) {}
  void writeMicroseconds(int) {}
};

// constrain macro
template<typename T>
inline T constrain(T v, T lo, T hi) { if (v < lo) return lo; if (v > hi) return hi; return v; }

// Simple String helper
inline String String_format(const char *fmt, ...) { return String(); }
