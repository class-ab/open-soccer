#include "include/subsystems/communication.h"

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

#include "include/subsystems/localization.h"
#include "include/subsystems/robot_config.h"
#include "include/subsystems/robot_state.h"
#include "include/subsystems/strategy.h"

// ============================================================
// RF24 Hardware Configuration
// ============================================================
// Using Teensy 4.1 SPI pins: MOSI=11, MISO=12, SCK=13
#define RF24_CE_PIN 34
#define RF24_CSN_PIN 36

RF24 radio(RF24_CE_PIN, RF24_CSN_PIN);

// Pipe addresses for bidirectional communication
const byte PIPE_ROBOT1[6] = "ROB1\0";
const byte PIPE_ROBOT2[6] = "ROB2\0";

// ============================================================
// Communication Packet Structures (optimized for size)
// ============================================================

// RobotPose packet: 21 bytes
struct RemoteRobotPosePacket {
  uint8_t valid;           // 1 byte
  float xMm;               // 4 bytes
  float yMm;               // 4 bytes
  float headingDeg;        // 4 bytes
  float quality;           // 4 bytes
  uint32_t timestampMs;    // 4 bytes
} __attribute__((packed));  // Total: 21 bytes

// FieldBall packet: 21 bytes
struct RemoteFieldBallPacket {
  uint8_t valid;           // 1 byte
  float xMm;               // 4 bytes
  float yMm;               // 4 bytes
  float distanceCm;        // 4 bytes
  float angleDeg;          // 4 bytes
  uint32_t timestampMs;    // 4 bytes
} __attribute__((packed));  // Total: 21 bytes

// OpponentRobot packet: 17 bytes per opponent
struct RemoteOpponentPacket {
  uint8_t valid;           // 1 byte
  float xMm;               // 4 bytes
  float yMm;               // 4 bytes
  float confidence;        // 4 bytes
  uint32_t timestampMs;    // 4 bytes
} __attribute__((packed));  // Total: 17 bytes

struct RemoteRobotStatePacket {
  uint8_t state;
  uint8_t damaged;
} __attribute__((packed));

// Combined payload: pose (21) + ball (21) + 3 opponents (51) = 93 bytes
// (nRF24 max payload is 32 bytes, so we'll use dynamic payload)
// Actually, we'll send in separate payloads to keep things simple

struct CommunicationPayload {
  uint8_t type;  // 0=pose, 1=ball, 2=opponent
  uint8_t data[31];
} __attribute__((packed));  // Total: 32 bytes

// ============================================================
// Global State
// ============================================================
namespace {
  uint8_t currentRobotNumber = 1;  // Selected robot (1 or 2)

  // Cached remote data
  RobotPose remoteRobotPose = {false, 0, 0, 0, 0, 0};
  LocalState remoteRobotState = {RobotState::attacking, RobotGoal::none};
  bool remoteRobotStateReceived = false;
  FieldBall remoteFieldBall = {false, 0, 0, 0, 0, 0};
  OpponentRobot remoteOpponents[5] = {};
  int remoteOpponentCount = 0;

  unsigned long lastPoseSendMs = 0;
  unsigned long lastStateSendMs = 0;
  unsigned long lastBallSendMs = 0;
  unsigned long lastOpponentsSendMs = 0;

  // Statistics
  uint32_t packetsReceived = 0;
  uint32_t packetsSent = 0;
  uint32_t sendFailures = 0;

  constexpr unsigned long POSE_SEND_INTERVAL_MS = 50;
  constexpr unsigned long STATE_SEND_INTERVAL_MS = 50;
  constexpr unsigned long BALL_SEND_INTERVAL_MS = 50;
  constexpr unsigned long OPPONENTS_SEND_INTERVAL_MS = 50;
  // radio.write() blocks until ACK/retries run out; pause TX after a failure so
  // a missing teammate cannot stall the control loop.
  constexpr unsigned long TX_FAILURE_BACKOFF_MS = 250;
  unsigned long txBackoffUntilMs = 0;
  bool txSentThisCall = false;
}

// ============================================================
// Helper Functions
// ============================================================

uint8_t getCurrentRobotNumber() {
  return currentRobotNumber;
}

void selectCurrentRobotNumber(uint8_t robotNumber) {
  if (robotNumber != 1 && robotNumber != 2) {
    return;
  }

  if (robotNumber != currentRobotNumber) {
    currentRobotNumber = robotNumber;
    Serial.print("Robot number changed to: ");
    Serial.println(currentRobotNumber);

    // Reconfigure radio pipes based on new robot number
    radio.stopListening();
    radio.flush_tx();
    radio.flush_rx();

    if (currentRobotNumber == 1) {
      radio.openWritingPipe(PIPE_ROBOT1);
      radio.openReadingPipe(1, PIPE_ROBOT2);
    } else {
      radio.openWritingPipe(PIPE_ROBOT2);
      radio.openReadingPipe(1, PIPE_ROBOT1);
    }

    radio.startListening();
  }
}

static void serializePose(const RobotPose &pose, uint8_t *buffer, int &len) {
  RemoteRobotPosePacket pkt;
  pkt.valid = pose.valid ? 1 : 0;
  pkt.xMm = pose.xMm;
  pkt.yMm = pose.yMm;
  pkt.headingDeg = pose.headingDeg;
  pkt.quality = pose.quality;
  pkt.timestampMs = pose.timestampMs;

  memcpy(buffer, &pkt, sizeof(pkt));
  len = sizeof(pkt);
}

static void deserializePose(const uint8_t *buffer, int len, RobotPose &pose) {
  if (len < (int)sizeof(RemoteRobotPosePacket)) {
    pose.valid = false;
    return;
  }

  RemoteRobotPosePacket pkt;
  memcpy(&pkt, buffer, sizeof(pkt));

  pose.valid = (pkt.valid != 0);
  pose.xMm = pkt.xMm;
  pose.yMm = pkt.yMm;
  pose.headingDeg = pkt.headingDeg;
  pose.quality = pkt.quality;
  pose.timestampMs = pkt.timestampMs;
}

static void serializeRobotState(const LocalState &state, uint8_t *buffer,
                                int &len) {
  RemoteRobotStatePacket pkt;
  pkt.state = static_cast<uint8_t>(state.robotState);
  pkt.damaged = (state.robotState == RobotState::damaged) ? 1 : 0;

  memcpy(buffer, &pkt, sizeof(pkt));
  len = sizeof(pkt);
}

static bool deserializeRobotState(const uint8_t *buffer, int len,
                                  LocalState &state) {
  if (len != (int)sizeof(RemoteRobotStatePacket)) {
    return false;
  }

  RemoteRobotStatePacket pkt;
  memcpy(&pkt, buffer, sizeof(pkt));

  if (pkt.damaged > 1) {
    return false;
  }

  if (pkt.damaged != 0 &&
      pkt.state == static_cast<uint8_t>(RobotState::damaged)) {
    state.robotState = RobotState::damaged;
  } else if (pkt.damaged == 0 &&
             pkt.state < static_cast<uint8_t>(RobotState::damaged)) {
    state.robotState = static_cast<RobotState>(pkt.state);
  } else {
    return false;
  }

  return true;
}

static void serializeBall(const FieldBall &ball, uint8_t *buffer, int &len) {
  RemoteFieldBallPacket pkt;
  pkt.valid = ball.valid ? 1 : 0;
  pkt.xMm = ball.xMm;
  pkt.yMm = ball.yMm;
  pkt.distanceCm = ball.distanceCm;
  pkt.angleDeg = ball.angleDeg;
  pkt.timestampMs = ball.timestampMs;

  memcpy(buffer, &pkt, sizeof(pkt));
  len = sizeof(pkt);
}

static void deserializeBall(const uint8_t *buffer, int len, FieldBall &ball) {
  if (len < (int)sizeof(RemoteFieldBallPacket)) {
    ball.valid = false;
    return;
  }

  RemoteFieldBallPacket pkt;
  memcpy(&pkt, buffer, sizeof(pkt));

  ball.valid = (pkt.valid != 0);
  ball.xMm = pkt.xMm;
  ball.yMm = pkt.yMm;
  ball.distanceCm = pkt.distanceCm;
  ball.angleDeg = pkt.angleDeg;
  ball.timestampMs = pkt.timestampMs;
}

static void serializeOpponents(const OpponentRobot *opponents, int count,
                               uint8_t *buffer, int &len) {
  int pos = 0;
  uint8_t opponentCount = (count > 5) ? 5 : count;

  // First byte: number of opponents
  buffer[pos++] = opponentCount;

  for (int i = 0; i < opponentCount; i++) {
    RemoteOpponentPacket pkt;
    pkt.valid = opponents[i].valid ? 1 : 0;
    pkt.xMm = opponents[i].xMm;
    pkt.yMm = opponents[i].yMm;
    pkt.confidence = opponents[i].confidence;
    pkt.timestampMs = opponents[i].timestampMs;

    if (pos + sizeof(pkt) <= 31) {
      memcpy(&buffer[pos], &pkt, sizeof(pkt));
      pos += sizeof(pkt);
    }
  }

  len = pos;
}

static void deserializeOpponents(const uint8_t *buffer, int len,
                                 OpponentRobot *opponents, int maxOpponents,
                                 int &count) {
  count = 0;

  if (len < 1) {
    return;
  }

  uint8_t opponentCount = buffer[0];
  if (opponentCount > (uint8_t)maxOpponents) {
    opponentCount = maxOpponents;
  }

  int pos = 1;
  for (int i = 0; i < opponentCount; i++) {
    if (pos + (int)sizeof(RemoteOpponentPacket) > len) {
      break;
    }

    RemoteOpponentPacket pkt;
    memcpy(&pkt, &buffer[pos], sizeof(pkt));
    pos += sizeof(pkt);

    opponents[i].valid = (pkt.valid != 0);
    opponents[i].xMm = pkt.xMm;
    opponents[i].yMm = pkt.yMm;
    opponents[i].confidence = pkt.confidence;
    opponents[i].timestampMs = pkt.timestampMs;

    count++;
  }
}

static void receiveData() {
  if (!radio.available()) {
    return;
  }

  uint8_t len = radio.getDynamicPayloadSize();
  if (len > 32) {
    len = 32;
  }

  uint8_t buffer[32];
  radio.read(buffer, len);
  packetsReceived++;

  if (len < 1) {
    return;
  }

  uint8_t type = buffer[0];
  uint8_t *data = &buffer[1];
  int dataLen = len - 1;

  switch (type) {
    case 0:  // Pose
      deserializePose(data, dataLen, remoteRobotPose);
      break;

    case 3:  // Robot state
      if (deserializeRobotState(data, dataLen, remoteRobotState)) {
        remoteRobotStateReceived = true;
      }
      break;

    case 1:  // Ball
      deserializeBall(data, dataLen, remoteFieldBall);
      break;

    case 2:  // Opponents
      deserializeOpponents(data, dataLen, remoteOpponents, 5, remoteOpponentCount);
      break;
  }
}

static void sendPacket(const uint8_t *buffer, int len, unsigned long now) {
  radio.stopListening();
  const bool ok = radio.write(buffer, len);
  radio.startListening();
  txSentThisCall = true;
  if (ok) {
    packetsSent++;
  } else {
    sendFailures++;
    txBackoffUntilMs = now + TX_FAILURE_BACKOFF_MS;
  }
}

static void transmitData(unsigned long now) {
  if (now < txBackoffUntilMs) {
    return;
  }
  txSentThisCall = false;

  // One packet per call; other due packets go out on following loops.
  if (now - lastPoseSendMs >= POSE_SEND_INTERVAL_MS) {
    lastPoseSendMs = now;

    RobotPose localPose;
    getRobotPose(localPose);

    uint8_t buffer[32];
    int len;
    serializePose(localPose, &buffer[1], len);

    buffer[0] = 0;  // Type: Pose
    len++;

    sendPacket(buffer, len, now);
  }

  if (!txSentThisCall && now - lastStateSendMs >= STATE_SEND_INTERVAL_MS) {
    LocalState localState;
    getLocalState(localState);

    // A stopped robot only publishes damage, not a pending role selection.
    if (robotCurrentlyRunning ||
        localState.robotState == RobotState::damaged) {
      lastStateSendMs = now;

      uint8_t buffer[32];
      int len;
      serializeRobotState(localState, &buffer[1], len);

      buffer[0] = 3;  // Type: Robot state
      len++;

      sendPacket(buffer, len, now);
    }
  }

  if (!txSentThisCall && now - lastBallSendMs >= BALL_SEND_INTERVAL_MS) {
    lastBallSendMs = now;

    FieldBall localBall;
    getFieldBall(localBall);

    uint8_t buffer[32];
    int len;
    serializeBall(localBall, &buffer[1], len);

    buffer[0] = 1;  // Type: Ball
    len++;

    sendPacket(buffer, len, now);
  }

  if (!txSentThisCall &&
      now - lastOpponentsSendMs >= OPPONENTS_SEND_INTERVAL_MS) {
    lastOpponentsSendMs = now;

    OpponentRobot opponents[5];
    int opponentCount = 0;
    getOpponents(opponents, 5, opponentCount);

    uint8_t buffer[32];
    int len;
    serializeOpponents(opponents, opponentCount, &buffer[1], len);

    buffer[0] = 2;  // Type: Opponents
    len++;

    sendPacket(buffer, len, now);
  }
}

// ============================================================
// Public Interface
// ============================================================

bool initCommunication() {
  Serial.println("Initializing RF24 communication...");

  if (!radio.begin()) {
    Serial.println("ERROR: nRF24L01+ not detected!");
    return false;
  }

  // Optimize for efficiency and reliability
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);  // Slower rate = more reliable = better range
  radio.setChannel(76);
  radio.enableDynamicPayloads();
  radio.setRetries(3, 3);  // 1 ms delay x 3 retries; default (5,15) can block ~35 ms
  // CRC configuration for reliability
  radio.setCRCLength(RF24_CRC_16);

  // Setup pipes
  currentRobotNumber = 1;  // Default to robot 1
  radio.openWritingPipe(PIPE_ROBOT1);
  radio.openReadingPipe(1, PIPE_ROBOT2);

  radio.startListening();

  Serial.print("RF24 initialized. Robot: ");
  Serial.println(currentRobotNumber);
  return true;
}

void updateCommunication() {
  unsigned long now = millis();

  // Receive data from other robot
  receiveData();

  // Transmit local data to other robot
  transmitData(now);
}

void getRemoteRobotPose(RobotPose &out) {
  out = remoteRobotPose;
}

bool getRemoteRobotState(LocalState &out) {
  out = remoteRobotState;
  return remoteRobotStateReceived;
}

void getRemoteFieldBall(FieldBall &out) {
  out = remoteFieldBall;
}

void getRemoteOpponents(OpponentRobot *out, int maxOpponents, int &count) {
  count = (remoteOpponentCount > maxOpponents) ? maxOpponents : remoteOpponentCount;
  if (count > 0) {
    memcpy(out, remoteOpponents, count * sizeof(OpponentRobot));
  }
}

void printCommunicationStats() {
  Serial.print("Communication Stats - Robot: ");
  Serial.print(currentRobotNumber);
  Serial.print(" | TX: ");
  Serial.print(packetsSent);
  Serial.print(" | RX: ");
  Serial.print(packetsReceived);
  Serial.print(" | Failures: ");
  Serial.println(sendFailures);
}
