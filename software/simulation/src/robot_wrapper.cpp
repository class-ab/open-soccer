#include "robot.h"
#include <thread>
#include <atomic>
#include <iostream>
#include "sim_hal/Arduino.h"

// forward-declare Arduino-style entrypoints from robot.cpp
extern void setup();
extern void loop();

static std::thread robot_thread;
static std::atomic<bool> robot_thread_running(false);
static std::atomic<bool> robot_request_stop(false);

static void robot_thread_func() {
  // Run the firmware setup once
  try {
    setup();
  } catch (...) {
    std::cerr << "Exception in robot setup()" << std::endl;
  }

  // Run loop() until requested to stop. The firmware may call delay(), which will block on
  // the simulator-driven condition variable implemented in sim_hal.
  while (!robot_request_stop.load()) {
    try {
      loop();
    } catch (...) {
      std::cerr << "Exception in robot loop()" << std::endl;
      break;
    }
    // If the robot code doesn't call delay() frequently, yield a little to avoid busy spin
    std::this_thread::yield();
  }
}

// Start the robot thread (calls setup() then repeatedly loop())
void robot_init() {
  if (robot_thread_running.load()) return;
  robot_request_stop.store(false);
  robot_thread = std::thread(robot_thread_func);
  robot_thread_running.store(true);
}

// Stop the robot thread and join
void robot_stop() {
  if (!robot_thread_running.load()) return;
  robot_request_stop.store(true);
  // notify simulator waiters in case robot thread is blocked inside delay()
  sim_set_millis(millis());
  if (robot_thread.joinable()) robot_thread.join();
  robot_thread_running.store(false);
}

// Existing tick API left in place (no-op for threaded model)
void robot_tick(unsigned long dtMs) {
  (void)dtMs;
}
