#include "include/subsystems/robot_state.h"
#include "include/subsystems/robot_config.h"
#include "include/subsystems/communication.h"
#include "include/subsystems/display.h"
#include "include/subsystems/strategy.h"

BallPacket latestBallPacket = {false, 0.0f, 0.0f, 0};
unsigned long lastBallPacketMs = 0;
uint8_t ballPacketBuf[BALL_PACKET_LEN];
uint8_t ballPacketIdx = 0;
bool ballSyncFound = false;

MoveProfile currentMoveProfile = {false, 0.0f, 0.0f, 0.0f, 0};

unsigned long bootMillis = 0;
unsigned long lastRunStateChangeMs = 0;
bool robotCurrentlyRunning = false;

unsigned long lastBatteryCheckMs = 0;
float lastBatteryVoltage = 0.0f;
bool shutdownLatched = false;
bool dribblerShouldRun = false;

unsigned long lastDisplayUpdateMs = 0;
bool displayAvailable = false;

bool button1State = false;
bool button2State = false;
bool button3State = false;

namespace { // start namespace
bool button1RawState = LOW;
bool button2RawState = LOW;
bool button3RawState = LOW;
unsigned long button1LastChangeMs = 0;
unsigned long button2LastChangeMs = 0;
unsigned long button3LastChangeMs = 0;

void updateButtonState(bool rawState, bool& lastRawState,
                       unsigned long& lastChangeMs, bool& stableState,
                       unsigned long now) {
    if (rawState != lastRawState) {
        lastRawState = rawState;
        lastChangeMs = now;
    }

    if (now - lastChangeMs >= BUTTON_DEBOUNCE_MS) {
        stableState = rawState;
    }
}
} // end namespace

float currentYawDeg = 0.0f;
float desiredHeadingDeg = 0.0f;
float headingIntegral = 0.0f;
float headingLastError = 0.0f;
unsigned long headingLastTimeMs = 0;
bool headingPidInitialized = false;

void checkButtons() {
    unsigned long now = millis();

    // External pull-downs hold released buttons LOW; pressing drives the pin HIGH.
    updateButtonState(digitalRead(button1) == HIGH, button1RawState,
                      button1LastChangeMs, button1State, now);
    updateButtonState(digitalRead(button2) == HIGH, button2RawState,
                      button2LastChangeMs, button2State, now);
    updateButtonState(digitalRead(button3) == HIGH, button3RawState,
                      button3LastChangeMs, button3State, now);

    static bool lastButton1State = false;
    if (button1State && !lastButton1State) {
        uint8_t nextRobot = getCurrentRobotNumber() == 1 ? 2 : 1;
        selectCurrentRobotNumber(nextRobot);
        selectLocalRobotRole(nextRobot);
        Serial.print("Selected robot ");
        Serial.print(nextRobot);
        Serial.println(nextRobot == 1 ? " (ATTACKER)" : " (DEFENDER)");
        updateDisplay();
    }
    lastButton1State = button1State;

    static bool lastButton2State = false;
    if (button2State && !lastButton2State) {
        robotCurrentlyRunning = !robotCurrentlyRunning;
        Serial.println(robotCurrentlyRunning ? "Robot enabled" : "Robot disabled");
        updateDisplay();
    }
    lastButton2State = button2State;

    static bool lastButton3State = false;
    if (button3State && !lastButton3State) {
        markLocalRobotDamaged();
        Serial.println("Local robot DAMAGED");
        updateDisplay();
    }
    lastButton3State = button3State;
}
