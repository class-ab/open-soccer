/*
  Teensy 4.1  ->  Aerostar RVS G2 20A BLDC ESC
  ---------------------------------------------
  Pin 23 = ESC throttle/control line
  Pin 17 = ESC reverse (RVS) line

  KEY POINT: Both lines are standard RC-style PWM signals (~50Hz,
  1000-2000us pulse widths) — NOT simple digital HIGH/LOW. That's why
  digitalWrite() didn't work before. The Servo library generates the
  correct pulse train for us.

  The reverse line behaves like a 2-position switch channel from an RC
  transmitter: one pulse-width range selects forward, the other selects
  reverse. Verify/tune the exact thresholds against the Aerostar LCD
  programming card or manual if this doesn't match your unit — values
  below use the common RC convention.
*/

#include <Servo.h>

// ---- Pin assignments ----
const int THROTTLE_PIN = 23;   // main ESC control line (speed)
const int REVERSE_PIN  = 17;   // ESC reverse/direction line

Servo escThrottle;
Servo escReverse;

// ---- Standard RC pulse widths (microseconds) ----
const int PULSE_MIN     = 1000;   // zero throttle / arm value
const int PULSE_NEUTRAL = 1500;
const int PULSE_MAX     = 2000;   // full throttle

// ---- Reverse-line "switch" positions ----
// Treat this exactly like a 2-position switch channel on a transmitter.
// Adjust these two values if your unit's threshold differs.
const int REVERSE_FORWARD_US = 1000;   // one direction
const int REVERSE_REVERSE_US = 2000;   // other direction

// ---- Run parameters ----
const int RUN_THROTTLE_US = 1200;   // gentle throttle above neutral; tune to taste
const int SPIN_TIME_MS    = 15000;   // how long to spin each direction
const int PAUSE_MS        = 1000;   // pause at zero throttle between direction changes

void armESC() {
  // Most ESCs require a steady "zero throttle" signal for a couple of
  // seconds right after power-up before they'll accept commands. Skipping
  // this is a common reason an ESC appears "dead" or won't respond.
  Serial.println("Arming ESC (holding zero throttle)...");
  escThrottle.writeMicroseconds(PULSE_MIN);
  delay(3000);
  Serial.println("Armed.");
}

void spinDirection(int reverseValue, const char* label) {
  Serial.print("Spinning: ");
  Serial.println(label);

  // Set direction first, give the ESC a moment to register the line
  escReverse.writeMicroseconds(reverseValue);
  delay(200);

  // Ramp isn't strictly required (RVS supports instant reversing) but a
  // simple direct jump to run speed is fine for a basic test.
  escThrottle.writeMicroseconds(RUN_THROTTLE_US);
  delay(SPIN_TIME_MS);

  // Always return to zero throttle before changing direction or stopping
  escThrottle.writeMicroseconds(PULSE_MIN);
  delay(PAUSE_MS);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  escThrottle.attach(THROTTLE_PIN);
  escReverse.attach(REVERSE_PIN);

  // Start the reverse line in a known state before arming
  escReverse.writeMicroseconds(REVERSE_FORWARD_US);

  armESC();
}

void loop() {
  spinDirection(REVERSE_REVERSE_US, "REVERSE");
}