#include "robot.h"
#include <thread>
#include <atomic>
#include <array>
#include <iostream>
#include "sim_hal/Arduino.h"
#include "sim_hal/sim_robot_io.h"

// forward-declare Arduino-style entrypoints from robot.cpp
extern void setup();
extern void loop();

namespace {
struct RobotThreadState {
  std::thread thread;
  std::atomic<bool> running{false};
  std::atomic<bool> stopRequested{false};
};
std::array<RobotThreadState, 2> g_robotThreads;

void robot_thread_func(int index) {
  sim_bind_robot_slot(index);
  // Run the firmware setup once
  try {
    setup();
  } catch (...) {
    std::cerr << "Exception in robot " << index << " setup()" << std::endl;
  }
  uint64_t generation = sim_robot_setup_complete(index);
  auto &state = g_robotThreads[index];
  while (!state.stopRequested.load()) {
    sim_wait_for_tick(generation);
    if (state.stopRequested.load()) break;
    sim_pull_robot_inputs(index);
    sim_apply_pending_role_command();
    try {
      loop();
    } catch (...) {
      std::cerr << "Exception in robot " << index << " loop()" << std::endl;
      break;
    }
    sim_push_robot_outputs(index);
    sim_complete_tick(index, generation);
  }
}
}  // namespace

// Start a robot thread (calls setup() then repeatedly loop()) for the given slot (0 or 1)
void robot_init(int index) {
  auto &state = g_robotThreads[index];
  if (state.running.load()) return;
  state.stopRequested.store(false);
  state.thread = std::thread(robot_thread_func, index);
  state.running.store(true);
}

// Stop the robot thread for the given slot and join
void robot_stop(int index) {
  auto &state = g_robotThreads[index];
  if (!state.running.load()) return;
  state.stopRequested.store(true);
  sim_request_robot_exit();
  sim_set_millis(millis());
  if (state.thread.joinable()) state.thread.join();
  state.running.store(false);
}

// Existing tick API left in place (no-op for threaded model)
void robot_tick(unsigned long dtMs) {
  (void)dtMs;
}

