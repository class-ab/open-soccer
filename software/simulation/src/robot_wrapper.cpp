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
  uint64_t generation = sim_robot_setup_complete();
  while (!robot_request_stop.load()) {
    sim_wait_for_tick(generation);
    if (robot_request_stop.load()) break;
    try {
      loop();
    } catch (...) {
      std::cerr << "Exception in robot loop()" << std::endl;
      break;
    }
    sim_complete_tick(generation);
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
  sim_request_robot_exit();
  sim_set_millis(millis());
  if (robot_thread.joinable()) robot_thread.join();
  robot_thread_running.store(false);
}

// Existing tick API left in place (no-op for threaded model)
void robot_tick(unsigned long dtMs) {
  (void)dtMs;
}
