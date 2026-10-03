#include <Arduino.h>
#include <Servo.h>

#include "include/subsystems/dribbler.h"
#include "include/subsystems/robot_config.h"
#include "include/subsystems/communication.h"

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
  pinMode(kicker, OUTPUT);
  pinMode(charge, OUTPUT);
  digitalWrite(kicker, LOW);
  digitalWrite(charge, LOW);
}

// Kickers are dead on both robots: charge and kick outputs are held LOW permanently.
void kick() {
  digitalWrite(kicker, LOW);
  digitalWrite(charge, LOW);
  kicking = false;
  /*
  if (getCurrentRobotNumber() == 2) {
    digitalWrite(kicker, LOW);
    digitalWrite(charge, LOW);
    kicking = false;
    return;
  }
  digitalWrite(charge, LOW);
  t0 = millis();
  kicking = true;
  */
}

void updateKicker() {
  digitalWrite(kicker, LOW);
  digitalWrite(charge, LOW);
  kicking = false;
  /*
  if (getCurrentRobotNumber() == 2) {
    digitalWrite(kicker, LOW);
    digitalWrite(charge, LOW);
    kicking = false;
    return;
  }

  if (!kicking) {
    digitalWrite(charge, HIGH);
    return;
  }
  unsigned long t = millis() - t0;

  if (t >= 50) {
    digitalWrite(kicker, LOW); 
    digitalWrite(charge, HIGH); 
    kicking = false; 
  } else if (t >= 32) {
    digitalWrite(kicker, LOW);
  } else if (t >= 20) {
    digitalWrite(kicker, HIGH);
  }
  */
}
