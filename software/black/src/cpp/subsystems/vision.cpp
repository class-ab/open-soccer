#include "include/subsystems/vision.h"

#include "include/subsystems/drivebase.h"
#include "include/subsystems/dribbler.h"
#include "include/subsystems/imu.h"
#include "include/subsystems/robot_config.h"
#include "include/subsystems/robot_state.h"

void initBallTracking() {
  BALL_UART.begin(BALL_UART_BAUD);
  Serial.println("Ball-tracking UART receiver ready");
}

bool isBallSyncByte(uint8_t b) {
  return (b == BALL_SYNC);
}

bool isYellowGoalSyncByte(uint8_t b) {
  return (b == YELLOW_GOAL_SYNC);
}

bool isBlueGoalSyncByte(uint8_t b) {
  return (b == BLUE_GOAL_SYNC);
}

void decodeBallPacket(const uint8_t *p) {
  latestBallPacket.detected = p[1] != 0;
  latestBallPacket.angleDeg = ((int16_t)((p[2] << 8) | p[3])) / 100.0f;
  latestBallPacket.distanceCM = (float)((uint16_t)((p[4] << 8) | p[5]));
  latestBallPacket.sizeByte = p[6];
  lastBallPacketMs = millis();
  if (latestBallPacket.detected) {
    lastBallSeenMs = millis();
  }

#ifdef DEBUG_BALL_LINK
  Serial.print("[BALL PKT OK] sync=A");
  Serial.print("  detected=");
  Serial.print(latestBallPacket.detected ? "YES" : "no ");
  Serial.print("  angle=");
  Serial.print(latestBallPacket.angleDeg, 2);
  Serial.print("deg  radius=");
  Serial.print(latestBallPacket.distanceCM, 0);
  Serial.print("px  size=");
  Serial.println(latestBallPacket.sizeByte);
#endif
}

void decodeYellowGoalPacket(const uint8_t *p) {
  latestYellowGoalPacket.detected = p[1] != 0;
  latestYellowGoalPacket.angleDeg = ((int16_t)((p[2] << 8) | p[3])) / 100.0f;
  latestYellowGoalPacket.distanceCM = (float)((uint16_t)((p[4] << 8) | p[5]));
  latestYellowGoalPacket.sizeByte = p[6];
  lastYellowGoalPacketMs = millis();

#ifdef DEBUG_BALL_LINK
  Serial.print("[YELLOW GOAL PKT OK] sync=B");
  Serial.print("  detected=");
  Serial.print(latestYellowGoalPacket.detected ? "YES" : "no ");
  Serial.print("  angle=");
  Serial.print(latestYellowGoalPacket.angleDeg, 2);
  Serial.print("deg  radius=");
  Serial.print(latestYellowGoalPacket.distanceCM, 0);
  Serial.print("px  size=");
  Serial.println(latestYellowGoalPacket.sizeByte);
#endif
}

void decodeBlueGoalPacket(const uint8_t *p) {
  latestBlueGoalPacket.detected = p[1] != 0;
  latestBlueGoalPacket.angleDeg = ((int16_t)((p[2] << 8) | p[3])) / 100.0f;
  latestBlueGoalPacket.distanceCM = (float)((uint16_t)((p[4] << 8) | p[5]));
  latestBlueGoalPacket.sizeByte = p[6];
  lastBlueGoalPacketMs = millis();

#ifdef DEBUG_BALL_LINK
  Serial.print("[BLUE GOAL PKT OK] sync=C");
  Serial.print("  detected=");
  Serial.print(latestBlueGoalPacket.detected ? "YES" : "no ");
  Serial.print("  angle=");
  Serial.print(latestBlueGoalPacket.angleDeg, 2);
  Serial.print("deg  radius=");
  Serial.print(latestBlueGoalPacket.distanceCM, 0);
  Serial.print("px  size=");
  Serial.println(latestBlueGoalPacket.sizeByte);
#endif
}

// Dispatch a fully-framed, checksum-valid 8-byte packet to the store that
// matches its sync byte.
void decodeVisionPacket(const uint8_t *p) {
  uint8_t sync = p[0];
  if (sync == BALL_SYNC) {
    decodeBallPacket(p);
  } else if (sync == YELLOW_GOAL_SYNC) {
    decodeYellowGoalPacket(p);
  } else if (sync == BLUE_GOAL_SYNC) {
    decodeBlueGoalPacket(p);
  } else {
#ifdef DEBUG_BALL_LINK
    Serial.println("[VISION DROP] bad sync byte");
#endif
  }
}

// The OpenMV sends three 8-byte packets (ball, yellow goal, blue goal) over
// the single hardware UART. We frame a stream of arbitrary incoming bytes:
// once any known sync byte is seen we collect BALL_PACKET_LEN bytes, verify
// the trailing XOR checksum, and dispatch only valid frames.
void processVisionPackets() {
  while (BALL_UART.available()) {
    uint8_t b = BALL_UART.read();

    if (!visionSyncFound) {
      if (isBallSyncByte(b) || isYellowGoalSyncByte(b) || isBlueGoalSyncByte(b)) {
        visionPacketBuf[0] = b;
        visionPacketIdx = 1;
        visionSyncFound = true;
      }
      continue;
    }

    visionPacketBuf[visionPacketIdx++] = b;

    if (visionPacketIdx == BALL_PACKET_LEN) {
      uint8_t checksum = 0;
      for (uint8_t i = 0; i < BALL_PACKET_LEN - 1; i++) {
        checksum ^= visionPacketBuf[i];
      }

      if (checksum == visionPacketBuf[BALL_PACKET_LEN - 1]) {
        decodeVisionPacket(visionPacketBuf);
      }
#ifdef DEBUG_BALL_LINK
      else {
        Serial.println("[VISION DROP] checksum mismatch");
      }
#endif
      visionSyncFound = false;
      visionPacketIdx = 0;
    }
  }
}

void getLatestBallData(BallPacket &out) {
  out = latestBallPacket;
}

void getLatestYellowGoalData(GoalPacket &out) {
  out = latestYellowGoalPacket;
}

void getLatestBlueGoalData(GoalPacket &out) {
  out = latestBlueGoalPacket;
}
