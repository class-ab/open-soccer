#include "battery.h"

#include "display.h"
#include "drivebase.h"
#include "robot_state.h"

// ============================================================
// Battery Monitor
// ============================================================

// Voltage divider: BATTERY+ --[4.7k]-- (A2 node) --[1k]-- GND
// Vnode = Vbatt * R2 / (R1 + R2)  =>  Vbatt = Vnode * (R1 + R2) / R2
static const int BATTERY_PIN = A2;
static const float BATTERY_DIVIDER_R1 = 4700.0f; // ohms, battery side
static const float BATTERY_DIVIDER_R2 = 1000.0f; // ohms, ground side
static const float BATTERY_DIVIDER_RATIO =
  (BATTERY_DIVIDER_R1 + BATTERY_DIVIDER_R2) / BATTERY_DIVIDER_R2; // 5.7

static const int ADC_RESOLUTION_BITS = 12;
static const int ADC_MAX_VALUE = (1 << ADC_RESOLUTION_BITS) - 1; // 4095
static const float ADC_REF_VOLTAGE = 3.3f; // Teensy 4.x ADC reference

// At the shutdown threshold (14.7V), the divider node sits at
// 14.7 / 5.7 = 2.58V - comfortably under the 3.3V Teensy ADC max even
// well above the cutoff (e.g. a full 4S charge at ~16.8V -> ~2.95V).
// const float BATTERY_SHUTDOWN_VOLTAGE = 14.7f;
static const float BATTERY_SHUTDOWN_VOLTAGE = 0.0f;
static const unsigned long BATTERY_CHECK_INTERVAL_MS = 5000;
static const int BATTERY_SAMPLE_COUNT = 8; // oversampled per reading, for stability

static unsigned long lastBatteryCheckMs = 0;
float lastBatteryVoltage = 0.0f;

// Latches true the moment a low-battery shutdown fires. Once set, the
// robot is intentionally dead until power-cycled - see emergencyShutdown()
// for why this isn't auto-resumable.
bool shutdownLatched = false;

static float readBatteryVoltage() {
  long sum = 0;

  for (int i = 0; i < BATTERY_SAMPLE_COUNT; i++) {
    sum += analogRead(BATTERY_PIN);
    delayMicroseconds(200);
  }

  float avgRaw = (float)sum / (float)BATTERY_SAMPLE_COUNT;
  float nodeVoltage = (avgRaw / (float)ADC_MAX_VALUE) * ADC_REF_VOLTAGE;

  return nodeVoltage * BATTERY_DIVIDER_RATIO;
}

// Reads the battery, stores it for the display, and immediately trips
// emergencyShutdown() if it's below BATTERY_SHUTDOWN_VOLTAGE.
void checkBattery() {
  lastBatteryVoltage = readBatteryVoltage();
  lastBatteryCheckMs = millis();

  if (lastBatteryVoltage < BATTERY_SHUTDOWN_VOLTAGE) {
    emergencyShutdown();
  }
}

void batteryTick() {
  if (millis() - lastBatteryCheckMs >= BATTERY_CHECK_INTERVAL_MS) {
    checkBattery(); // may call emergencyShutdown() and never return
  }
}

void initBattery() {
  analogReadResolution(ADC_RESOLUTION_BITS);

  // Take an immediate battery reading before anything else spins up, so
  // a dead/miswired/already-too-low battery is caught (and shuts things
  // down) before the BNO08x or motors are ever touched.
  checkBattery();
}

// Immediately halts the robot: stops every motor, latches the shutdown
// state, shows a permanent warning screen, and then hangs forever.
//
// This is intentionally NOT auto-resumable. A battery that has sagged
// below a safe under-voltage threshold usually keeps sagging further
// under load, and RCJ Soccer fields have no supervised "safe to
// continue" state to fall back to - the only reliable safe state is
// fully off, requiring a deliberate power cycle (with a charged/swapped
// battery) to clear.
void emergencyShutdown() {
  stopAllMotors();
  shutdownLatched = true;

  unsigned long uptimeMs = millis() - bootMillis;

  Serial.print("EMERGENCY SHUTDOWN - battery ");
  Serial.print(lastBatteryVoltage, 2);
  Serial.println("V");

  showShutdownScreen(lastBatteryVoltage, BATTERY_SHUTDOWN_VOLTAGE, uptimeMs);

  while (true) {
    // Belt-and-braces: keep re-asserting motors off forever. Nothing
    // else in the program runs again after this point.
    stopAllMotors();
    delay(200);
  }
}
