#include "sim_hal/Arduino.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <mutex>
#include <condition_variable>
#include <atomic>

using namespace std::chrono;

static std::mutex serial_mutex;

HardwareSerial Serial;
HardwareSerial Serial7;
TwoWire Wire2;

// Simulator-controlled time and synchronization primitives
static std::mutex sim_time_mutex;
static std::condition_variable sim_time_cv;
static unsigned long sim_millis = 0; // simulator-driven epoch (ms)
static std::atomic<bool> sim_robot_exit(false);
static std::atomic<bool> sim_robot_enabled(true);

void HardwareSerial::begin(unsigned long) { /* no-op */ }

unsigned long millis() {
  std::lock_guard<std::mutex> lk(sim_time_mutex);
  return sim_millis;
}

void delay(unsigned long ms) {
  // Non-blocking with respect to real time: wait until the simulator advances simulated time past the target.
  std::unique_lock<std::mutex> lk(sim_time_mutex);
  unsigned long target = sim_millis + ms;
  // Wait until sim_millis >= target or robot is asked to exit or disabled (disabled will also block)
  sim_time_cv.wait(lk, [&]{ return sim_millis >= target || sim_robot_exit.load() || !sim_robot_enabled.load(); });
  // If robot was disabled, block here until enabled or exit
  while (!sim_robot_enabled.load() && !sim_robot_exit.load()) {
    sim_time_cv.wait(lk);
  }
}

void delayMicroseconds(unsigned int us) {
  // granularity will be milliseconds; convert and call delay
  if (us >= 1000u) delay(us / 1000u);
}

void pinMode(int pin, int mode) {
  (void)pin; (void)mode; // no-op
}

int digitalRead(int pin) {
  (void)pin; return LOW;
}

void digitalWrite(int pin, int value) {
  (void)pin; (void)value; // no-op
}

int analogRead(int pin) {
  (void)pin; // return mid-scale
  return 2048;
}

void analogWrite(int pin, int value) {
  (void)pin; (void)value; // no-op
}

void analogReadResolution(int bits) {
  (void)bits;
}

// Simulator control: set the simulated epoch (in ms) and notify any waiting robot thread(s)
void sim_set_millis(unsigned long ms) {
  {
    std::lock_guard<std::mutex> lk(sim_time_mutex);
    sim_millis = ms;
  }
  sim_time_cv.notify_all();
}

void sim_set_robot_enabled(bool enabled) {
  sim_robot_enabled.store(enabled);
  sim_time_cv.notify_all();
}

void sim_request_robot_exit() {
  sim_robot_exit.store(true);
  sim_time_cv.notify_all();
}

// HardwareSerial methods that print to console
void HardwareSerial::print(const std::string &s) {
  std::lock_guard<std::mutex> lock(serial_mutex);
  std::cout << s;
}
void HardwareSerial::print(float v, int precision) {
  std::lock_guard<std::mutex> lock(serial_mutex);
  std::cout.setf(std::ios::fixed); std::cout.precision(precision);
  std::cout << v;
}
void HardwareSerial::println(const std::string &s) {
  std::lock_guard<std::mutex> lock(serial_mutex);
  std::cout << s << std::endl;
}
void HardwareSerial::println(float v) {
  std::lock_guard<std::mutex> lock(serial_mutex);
  std::cout << v << std::endl;
}
void HardwareSerial::println(int v) {
  std::lock_guard<std::mutex> lock(serial_mutex);
  std::cout << v << std::endl;
}
void HardwareSerial::println(unsigned long v) {
  std::lock_guard<std::mutex> lock(serial_mutex);
  std::cout << v << std::endl;
}
void HardwareSerial::println() {
  std::lock_guard<std::mutex> lock(serial_mutex);
  std::cout << std::endl;
}
int HardwareSerial::available() { return 0; }
int HardwareSerial::read() { return -1; }
void HardwareSerial::write(uint8_t) { }
