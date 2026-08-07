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

void decodeBallPacket(const uint8_t *p) {
  uint8_t sync = p[0];

  if (sync != BALL_SYNC) {
#ifdef DEBUG_BALL_LINK
    Serial.println("[BALL DROP] color not chased (or bad sync byte)");
#endif
    return;
  }

  latestBallPacket.detected = p[1] != 0;
  latestBallPacket.angleDeg = ((int16_t)((p[2] << 8) | p[3])) / 100.0f;
  latestBallPacket.radiusPx = (float)((uint16_t)((p[4] << 8) | p[5]));
  latestBallPacket.sizeByte = p[6];
  lastBallPacketMs = millis();

#ifdef DEBUG_BALL_LINK
  Serial.print("[BALL PKT OK] sync=");
  Serial.print(sync == BALL_SYNC ? "A" : "B");
  Serial.print("  detected=");
  Serial.print(latestBallPacket.detected ? "YES" : "no ");
  Serial.print("  angle=");
  Serial.print(latestBallPacket.angleDeg, 2);
  Serial.print("deg  radius=");
  Serial.print(latestBallPacket.radiusPx, 0);
  Serial.print("px  size=");
  Serial.println(latestBallPacket.sizeByte);
#endif
}

void processBallPacket() {
  while (BALL_UART.available()) {
    uint8_t b = BALL_UART.read();

    if (!ballSyncFound) {
      if (isBallSyncByte(b)) {
        ballPacketBuf[0] = b;
        ballPacketIdx = 1;
        ballSyncFound = true;
      }
      continue;
    }

    ballPacketBuf[ballPacketIdx++] = b;

    if (ballPacketIdx == BALL_PACKET_LEN) {
      uint8_t checksum = 0;
      for (uint8_t i = 0; i < BALL_PACKET_LEN - 1; i++) {
        checksum ^= ballPacketBuf[i];
      }

      if (checksum == ballPacketBuf[BALL_PACKET_LEN - 1]) {
        decodeBallPacket(ballPacketBuf);
      }
#ifdef DEBUG_BALL_LINK
      else {
        Serial.println("[BALL DROP] checksum mismatch");
      }
#endif
      ballSyncFound = false;
      ballPacketIdx = 0;
    }
  }
}

void getLatestBallData(BallPacket &out) {
  out = latestBallPacket;
}

void chaseTick() {
  BallPacket ball;
  getLatestBallData(ball);

  unsigned long age_ms = millis() - lastBallPacketMs;
  bool haveFreshBall = ball.detected && (age_ms <= BALL_DATA_TIMEOUT_MS);

  if (!haveFreshBall) {
    float rotation = headingCorrection();
    drive(0.0f, 0.0f, rotation);
    stopAllDriveMotors();

#ifdef DEBUG_BALL_CHASE
    static unsigned long lastDebugMsA = 0;
    unsigned long nowMsA = millis();
    if (nowMsA - lastDebugMsA >= 100) {
      lastDebugMsA = nowMsA;
      Serial.print("chaseTick: no fresh ball data, ageMs=");
      Serial.println(age_ms);
    }
#endif
    return;
  }

  float chassisRelativeBallAngle = ball.angleDeg + CAMERA_MOUNT_OFFSET_DEG;

  float effectiveYaw = YAW_SIGN * currentYawDeg;
  desiredHeadingDeg = effectiveYaw + chassisRelativeBallAngle;

  float rotation = -headingCorrection();

  float radiusError = ball.radiusPx - BALL_TARGET_RADIUS_PX;
  float speed;

  if (radiusError <= 0.0f) {
    speed = 0.0f;
  } else {
    float t = constrain(radiusError / BALL_CHASE_RAMP_RANGE_PX, 0.0f, 1.0f);
    speed = BALL_CHASE_MIN_SPEED + t * (BALL_CHASE_MAX_SPEED - BALL_CHASE_MIN_SPEED);
  }

  float fieldDirection = effectiveYaw - chassisRelativeBallAngle;

  drive(fieldDirection, speed, rotation);

#ifdef DEBUG_BALL_CHASE
  static unsigned long lastDebugMsB = 0;
  unsigned long nowMsB = millis();
  if (nowMsB - lastDebugMsB >= 100) {
    lastDebugMsB = nowMsB;
    Serial.print("bearing(chassis)=");
    Serial.print(chassisRelativeBallAngle);
    Serial.print(" radiusPx=");
    Serial.print(ball.radiusPx);
    Serial.print(" size=");
    Serial.print(ball.sizeByte);
    Serial.print(" speed=");
    Serial.print(speed);
    Serial.print(" ageMs=");
    Serial.println(age_ms);
  }
#endif
}
