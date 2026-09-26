#include "sim_hal/sim_robot_io.h"

#include <mutex>

#include "subsystems/robot_state.h"

// Slot each robot thread is bound to; set once at thread start and then only
// ever read by that same thread, so no synchronization is needed for it.
thread_local int g_simRobotSlot = 0;

void sim_bind_robot_slot(int slot) {
  g_simRobotSlot = slot;
}

namespace {
std::mutex g_bridgeMutex;

LocalState g_publishedState[2] = {
    {RobotState::attacking, RobotGoal::none},
    {RobotState::attacking, RobotGoal::none}};

SimRoleCommand g_pendingRole[2] = {SimRoleCommand::none, SimRoleCommand::none};

struct RobotInput {
  BallPacket ballPacket{false, 0.0f, 0.0f, 0};
  unsigned long lastBallPacketMs = 0;
  bool running = false;
};
RobotInput g_inputs[2];

struct RobotOutput {
  MoveProfile moveProfile{false, 0.0f, 0.0f, 0.0f, 0};
  bool dribblerShouldRun = false;
};
RobotOutput g_outputs[2];
}  // namespace

void sim_get_published_local_state(int slot, LocalState &out) {
  std::lock_guard<std::mutex> lock(g_bridgeMutex);
  out = g_publishedState[slot];
}

void sim_request_role(int slot, SimRoleCommand cmd) {
  std::lock_guard<std::mutex> lock(g_bridgeMutex);
  g_pendingRole[slot] = cmd;
}

void sim_apply_pending_role_command() {
  SimRoleCommand cmd;
  {
    std::lock_guard<std::mutex> lock(g_bridgeMutex);
    cmd = g_pendingRole[g_simRobotSlot];
    g_pendingRole[g_simRobotSlot] = SimRoleCommand::none;
  }
  if (cmd == SimRoleCommand::attack) {
    selectLocalRobotRole(1);
  } else if (cmd == SimRoleCommand::defend) {
    selectLocalRobotRole(2);
  } else if (cmd == SimRoleCommand::damage) {
    markLocalRobotDamaged();
  }
}

void sim_set_running(int slot, bool running) {
  std::lock_guard<std::mutex> lock(g_bridgeMutex);
  g_inputs[slot].running = running;
}

void sim_publish_ball_packet(int slot, const BallPacket &packet, unsigned long timestampMs) {
  std::lock_guard<std::mutex> lock(g_bridgeMutex);
  g_inputs[slot].ballPacket = packet;
  g_inputs[slot].lastBallPacketMs = timestampMs;
}

void sim_get_move_profile(int slot, MoveProfile &profile, bool &dribblerShouldRun) {
  std::lock_guard<std::mutex> lock(g_bridgeMutex);
  profile = g_outputs[slot].moveProfile;
  dribblerShouldRun = g_outputs[slot].dribblerShouldRun;
}

// The following two run on the owning firmware thread, so directly touching
// the (thread-local) robot_state globals here reads/writes that thread's own
// copy, exactly as if the firmware itself had done so.
void sim_pull_robot_inputs(int slot) {
  std::lock_guard<std::mutex> lock(g_bridgeMutex);
  latestBallPacket = g_inputs[slot].ballPacket;
  lastBallPacketMs = g_inputs[slot].lastBallPacketMs;
  robotCurrentlyRunning = g_inputs[slot].running;
}

void sim_push_robot_outputs(int slot) {
  LocalState state;
  getLocalState(state);
  std::lock_guard<std::mutex> lock(g_bridgeMutex);
  g_outputs[slot].moveProfile = currentMoveProfile;
  g_outputs[slot].dribblerShouldRun = dribblerShouldRun;
  g_publishedState[slot] = state;
}
