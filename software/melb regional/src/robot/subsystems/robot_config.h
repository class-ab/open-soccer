#ifndef SUBSYSTEMS_ROBOT_CONFIG_H
#define SUBSYSTEMS_ROBOT_CONFIG_H

// ============================================================
// Build-time debug toggles
// ============================================================
// Uncomment whichever of these you need, reflash, and watch Serial.
// Leave everything off for normal/competition runs - all three print
// often enough to noticeably slow things down and clutter the console.

// Print direction/yaw/vx/vy/wheel-speed diagnostics (about 10x/sec)
// during every drivebase move() - useful for confirming the YAW_SIGN
// convention and checking whether vx/vy are doing what you expect as
// the robot rotates. See drivebase.cpp's drive().
// #define DEBUG_MOVE

// Print ball-chase diagnostics (bearing, radius, size, computed speed,
// packet age) about 10x/sec while chaseTick() runs - useful for
// calibrating CAMERA_ROTATION_OFFSET_DEG (on the OpenMV side),
// CAMERA_MOUNT_OFFSET_DEG, BALL_TARGET_RADIUS_PX, and
// BALL_CHASE_RAMP_RANGE_PX. See ball_tracking.cpp's chaseTick().
#define DEBUG_BALL_CHASE

// Print raw ball-link diagnostics (per-packet decode results and
// checksum failures) - useful for debugging the physical link itself
// (wiring, clock rate, sync/checksum issues), independent of the
// ball-chase behavior above. Off by default since it prints at packet
// rate, which is a lot busier than the throttled DEBUG_BALL_CHASE
// output. See ball_tracking.cpp's processBallPacket().
// #define DEBUG_BALL_LINK

#endif
