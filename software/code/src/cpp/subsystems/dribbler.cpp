#include <Arduino.h>
#include <Servo.h>

#include "include/subsystems/dribbler.h"
#include "include/subsystems/robot_config.h"

static Servo escThrottle;
static Servo escReverse;

unsigned long t0;
bool kicking = false;

static void armESC() {
  Serial.println("Arming dribbler ESC (holding zero throttle)...");
  escThrottle.writeMicroseconds(DRIBBLER_PULSE_MIN);
  delay(3000);
  Serial.println("Dribbler ESC armed.");
}

void initDribbler() {
  escThrottle.attach(DRIBBLER_THROTTLE_PIN);
  escReverse.attach(DRIBBLER_REVERSE_PIN);
  escReverse.writeMicroseconds(DRIBBLER_FORWARD_US);
  armESC();
}

void setDribblerDirectionForward() {
  escReverse.writeMicroseconds(DRIBBLER_FORWARD_US);
}

void setDribblerDirectionReverse() {
  escReverse.writeMicroseconds(DRIBBLER_REVERSE_US);
}

void setDribblerThrottle(int throttleUs) {
  throttleUs = constrain(throttleUs, DRIBBLER_PULSE_MIN, DRIBBLER_PULSE_MAX);
  escThrottle.writeMicroseconds(throttleUs);
}

void stopDribbler() {
  escThrottle.writeMicroseconds(DRIBBLER_PULSE_MIN);
}

void initKicker() {
  digitalWrite(kicker, LOW);
  digitalWrite(charge, HIGH);
}

void kick() {
  digitalWrite(charge, LOW);
  t0 = millis();
  kicking = true;
}

void updateKicker() {
  if (!kicking) return;
  unsigned long t = millis() - t0;

  if (t >= 60) { // delay 60 - 40 = 20
    digitalWrite(kicker, LOW); 
    digitalWrite(charge, HIGH); 
    kicking = false; 
  } else if (t >= 40) { // delay 40 - 20 = 20
    digitalWrite(kicker, LOW);
  } else if (t >= 20) { // delay 20
    digitalWrite(kicker, HIGH);
  }
}
