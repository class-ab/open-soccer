#include <Arduino.h>
#include <Wire.h>

#include "include/robot.h"
#include "include/subsystems/vision.h"
#include "include/subsystems/battery.h"
#include "include/subsystems/communication.h"
#include "include/subsystems/display.h"
#include "include/subsystems/drivebase.h"
#include "include/subsystems/dribbler.h"
#include "include/subsystems/imu.h"
#include "include/subsystems/localization.h"
#include "include/subsystems/robot_config.h"
#include "include/subsystems/robot_state.h"
#include "include/subsystems/robot_tick.h"
#include "include/subsystems/strategy.h"

namespace {
void haltBoot(const char *device, const char *check) {
  Serial.print("BOOT FAILED: ");
  Serial.println(device);
  showBootStatus("BOOT FAILED", device);
  delay(1200);
  showBootStatus(device, check);

  // Do not enter loop(): it would try to use subsystems that did not start.
  // Keep every output stopped while the error remains visible on the OLED.
  stopAllDriveMotors();
  stopDribbler();
  while (true) {
    delay(250);
  }
}
}

void setup() {
  Serial.begin(115200);

  pinMode(M1a, OUTPUT);
  pinMode(M1b, OUTPUT);
  pinMode(M2a, OUTPUT);
  pinMode(M2b, OUTPUT);
  pinMode(M3a, OUTPUT);
  pinMode(M3b, OUTPUT);
  pinMode(M4a, OUTPUT);
  pinMode(M4b, OUTPUT);

  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(button3, INPUT);

  delay(100);

  Serial.println("Starting...");

  bootMillis = millis();
  lastRunStateChangeMs = bootMillis;

  Wire2.begin();
  Serial.println("BOOT 1/6: display");
  initDisplay();
  showBootStatus("BOOT 2/6", "Ball UART");
  initBallTracking();
  showBootStatus("BOOT 3/6", "LiDAR UART");
  initLocalization();
  showBootStatus("BOOT 4/6", "RF24 radio");
  if (!initCommunication()) {
    haltBoot("RF24 NOT FOUND", "Check SPI, CE/CSN");
  }

  showBootStatus("BOOT 5/6", "Dribbler ESC");
  initDribbler();
  setDribblerDirectionReverse();

  analogReadResolution(ADC_RESOLUTION_BITS);

  lastBatteryCheckMs = millis();
  checkBattery();

  showBootStatus("BOOT 6/6", "BNO08x IMU");
  if (!initIMU()) {
    haltBoot("BNO08X NOT FOUND", "Check I2C/power");
  }

  Serial.print("Initial Heading: ");
  Serial.println(currentYawDeg);
  Serial.println("BOOT COMPLETE");
}

void loop() {
  // main update system
  unsigned long now = millis();
  checkButtons();
  updateIMU();
  checkEnabledButton(now);
  processBallPacket();
  updateLocalization();
  updateCommunication();
  updateStrategy();
  
  if (robotCurrentlyRunning) {
    move();
  } else {
    stopAllMotors();
  }

  if (now - lastBatteryCheckMs >= BATTERY_CHECK_INTERVAL_MS) {
    checkBattery(); // check battery at time interval
  }

  if (now - lastDisplayUpdateMs >= DISPLAY_UPDATE_INTERVAL_MS) {
    lastDisplayUpdateMs = now;
    updateDisplay(); // update display at time interval
  }

  if (shutdownLatched) {
    return; // undervoltage check
  }
}

void checkEnabledButton(unsigned long now) {
  static bool lastButton1State = LOW;
  static bool clickPending = false;
  static unsigned long firstClickMs = 0;
  static uint8_t nextRobot = 1;

  const bool button1Pressed = button1State == HIGH && lastButton1State == LOW;
  if (button1Pressed) {
    if (clickPending && now - firstClickMs <= BUTTON_DOUBLE_CLICK_MS) {
      LocalState localState;
      getLocalState(localState);

      if (localState.robotState == RobotState::damaged) {
        selectCurrentRobotNumber(nextRobot);
        selectLocalRobotRole(nextRobot);
        Serial.print("Local robot restored as robot ");
        Serial.println(nextRobot);
        nextRobot = (nextRobot == 1) ? 2 : 1;
      } else {
        markLocalRobotDamaged();
        Serial.println("Local robot DAMAGED");
      }

      clickPending = false;
      updateDisplay();
    } else {
      // Wait for the double-click window to expire before treating this as a
      // role-selection click, so a double-click never briefly changes roles.
      clickPending = true;
      firstClickMs = now;
    }
  }

  if (clickPending && now - firstClickMs > BUTTON_DOUBLE_CLICK_MS) {
    selectCurrentRobotNumber(nextRobot);
    selectLocalRobotRole(nextRobot);
    Serial.print("Selected robot ");
    Serial.print(nextRobot);
    Serial.println(nextRobot == 1 ? " (ATTACKER)" : " (DEFENDER)");
    nextRobot = (nextRobot == 1) ? 2 : 1;
    clickPending = false;
    updateDisplay();
  }

  lastButton1State = button1State;

}

void stopAllMotors() {
  stopAllDriveMotors();
  stopDribbler();
}
