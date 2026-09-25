#include "include/subsystems/display.h"

#include <Wire.h>
#include <string>
#include <iostream>
#include <magic_enum.hpp>

#include "include/subsystems/robot_config.h"
#include "include/subsystems/robot_state.h"
#include "include/subsystems/localization.h"
#include "include/subsystems/communication.h"
#include "include/subsystems/strategy.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire2, OLED_RESET_PIN);

void initDisplay() {
  displayAvailable = display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS);

  if (!displayAvailable) {
    Serial.println("SSD1306 not found!");
    return;
  }

  display.setRotation(2);
  display.setTextColor(SSD1306_WHITE);
  showBootStatus("Booting...", "Display ready");
}

void showBootStatus(const char *line1, const char *line2) {
  if (!displayAvailable) {
    return;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(line1);
  if (line2 != nullptr) {
    display.println(line2);
  }
  display.display();
}

void updateDisplay() {
  if (!displayAvailable || shutdownLatched) {
    return;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  display.print("Batt:   ");
  display.print(lastBatteryVoltage, 2);
  display.println("V");

  /* ROBOT TIMER
  unsigned long now = millis();
  unsigned long uptimeMs = now - bootMillis; 
  unsigned long runStateMs = now - lastRunStateChangeMs;
  display.print("Uptime: ");
  display.println(formatDuration(uptimeMs));

  

  display.print(robotCurrentlyRunning ? "Run tmr: " : "Idle tmr:");
  display.println(formatDuration(runStateMs));
  */
  display.print("Status: ");
  display.println(robotCurrentlyRunning ? "RUNNING" : "STOPPED");

  display.print("LclPos: ");
  RobotPose pose;
  getRobotPose(pose);
  if (pose.valid) {
    display.print("X=" );
    display.println(pose.xMm, 0);
    display.print(" Y=" );
    display.println(pose.yMm, 0);
    display.print(" H= ");
    display.println(pose.headingDeg, 1);
  } else {
    display.println("INVALID");
  }
  /* display.print("RemPos: ");
  RobotPose remotePose;
  getRemoteRobotPose(remotePose);
  if (pose.valid) {
    display.print("X=" );
    display.print(pose.xMm, 0);
    display.print(" Y=" );
    display.print(pose.yMm, 0);
    display.print(" H= ");
    display.print(pose.headingDeg, 1);
  } else {
    display.println("INVALID");
  }
  */
 // get localState from strategy
  LocalState localState;
  getLocalState(localState);
  std::string_view printRobotState = magic_enum::enum_name(localState.robotState);
  std::string_view printRobotGoal = magic_enum::enum_name(localState.robotGoal);
  // get ballState from strategy
  ballLocation ballState;
  getBallState(ballState);
  std::string_view printBallState = magic_enum::enum_name(ballState.ballState);
  std::string_view printBallPossession = magic_enum::enum_name(ballState.ballPossession);
  // update display
  display.print("State: ");
  display.println(printRobotState.data());
  display.print("Goal: ");
  display.println(printRobotGoal.data());
  display.display();

  #ifdef LOCAL_STATE // update serial 
    Serial.print(printRobotState.data());
    Serial.print(" ");
    Serial.print(printRobotGoal.data());
    Serial.print(" ");
    Serial.print(printBallState.data());
    Serial.print(" ");
    Serial.print(printBallPossession.data());
    Serial.println(" ");
  #endif
}

String formatDuration(unsigned long ms) {
  unsigned long totalSeconds = ms / 1000UL;
  unsigned long hours = totalSeconds / 3600UL;
  unsigned long minutes = (totalSeconds % 3600UL) / 60UL;
  unsigned long seconds = totalSeconds % 60UL;

  char buf[16];

  if (hours > 0) {
    snprintf(buf, sizeof(buf), "%lu:%02lu:%02lu", hours, minutes, seconds);
  } else {
    snprintf(buf, sizeof(buf), "%02lu:%02lu", minutes, seconds);
  }

  return String(buf);
}
