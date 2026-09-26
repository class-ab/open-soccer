#pragma once

#include "subsystems/localization.h"
#include "subsystems/strategy.h"
#include "subsystems/robot_state.h"

// Two robots are simulated; each firmware instance runs on its own thread and
// is "bound" to a slot (0 or 1) once at thread start via sim_bind_robot_slot.
// All cross-thread state below is keyed by slot and mutex-protected: the
// render thread addresses a specific slot explicitly, while a robot's own
// firmware thread implicitly addresses its own slot via g_simRobotSlot.
extern thread_local int g_simRobotSlot;
void sim_bind_robot_slot(int slot);

// Localization: written by the render thread, read by the owning firmware
// thread (and by the *other* firmware thread for simulated radio comms).
void sim_set_localization(int slot, const RobotPose &pose, const FieldBall &ball,
                          const OpponentRobot *opponents, int count);
void sim_get_localization(int slot, RobotPose &pose, FieldBall &ball);

// IMU: written by the render thread from physical heading/yaw-rate.
void sim_set_imu_state(int slot, float headingDeg, float yawRateDegPerSec);

// Kicker: requested by the firmware thread, consumed by the render thread.
void sim_request_kick(int slot);
bool sim_consume_kick_request(int slot);

// Published firmware LocalState (role + current goal) for HUD display and
// for the simulated radio link to report "remote" state to the other robot.
void sim_get_published_local_state(int slot, LocalState &out);

// HUD-issued role/damage command, applied by the owning firmware thread at
// the start of its next tick (mirrors the real checkButtons() flow).
enum class SimRoleCommand { none, attack, defend, damage };
void sim_request_role(int slot, SimRoleCommand cmd);
void sim_apply_pending_role_command();

// Cross-thread bridge for the globals that would otherwise need to be shared
// between the render thread and a specific robot's (thread-local) firmware
// state: ball sensor input + run flag in, MoveProfile/dribbler flag out.
void sim_set_running(int slot, bool running);
void sim_publish_ball_packet(int slot, const BallPacket &packet, unsigned long timestampMs);
void sim_get_move_profile(int slot, MoveProfile &profile, bool &dribblerShouldRun);

// Called once per tick from robot_wrapper.cpp, on the owning firmware thread.
void sim_pull_robot_inputs(int slot);
void sim_push_robot_outputs(int slot);