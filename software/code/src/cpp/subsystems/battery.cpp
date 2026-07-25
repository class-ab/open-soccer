#include "include/subsystems/battery.h"

#include "include/subsystems/display.h"
#include "include/subsystems/drivebase.h"
#include "include/subsystems/robot_config.h"
#include "include/subsystems/robot_state.h"

float readBatteryVoltage() {
  long sum = 0;

  for (int i = 0; i < BATTERY_SAMPLE_COUNT; i++) {
    sum += analogRead(BATTERY_PIN);
    delayMicroseconds(200);
  }

  float avgRaw = (float)sum / (float)BATTERY_SAMPLE_COUNT;
  float nodeVoltage = (avgRaw / (float)ADC_MAX_VALUE) * ADC_REF_VOLTAGE;

  return nodeVoltage * BATTERY_DIVIDER_RATIO;
}

void checkBattery() {
  lastBatteryVoltage = readBatteryVoltage();
  lastBatteryCheckMs = millis();

  if (lastBatteryVoltage < BATTERY_SHUTDOWN_VOLTAGE) {
    emergencyShutdown();
  }
}

void emergencyShutdown() {
  stopAllMotors();
  shutdownLatched = true;

  unsigned long uptimeMs = millis() - bootMillis;

  Serial.print("EMERGENCY SHUTDOWN - battery ");
  Serial.print(lastBatteryVoltage, 2);
  Serial.println("V");

  if (displayAvailable) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("!! LOW BATTERY !!");
    display.println("   SHUTDOWN");
    display.println("");
    display.print("Batt:  ");
    display.print(lastBatteryVoltage, 2);
    display.println("V");
    display.print("Limit: ");
    display.print(BATTERY_SHUTDOWN_VOLTAGE, 1);
    display.println("V");
    display.print("Uptime: ");
    display.println(formatDuration(uptimeMs));
    display.println("");
    display.println("Power cycle to reset");
    display.display();
  }

  while (true) {
    stopAllMotors();
    delay(200);
  }
}
