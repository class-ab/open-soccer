const int charge = 31;  // CHARGE
const int kick = 32;    // KICK

const unsigned long pulseTime = 20;      // ms
const unsigned long switchDelay = 100;   // ms

void setup() {
  pinMode(charge, OUTPUT);
  pinMode(kick, OUTPUT);

  digitalWrite(charge, LOW);
  digitalWrite(kick, LOW);

  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'f') {
      Serial.println("RECEIVED - 5");
      digitalWrite(charge, HIGH);
      delay(1000);
      Serial.println("4");
      delay(1000);
      Serial.println("3");
      delay(1000);
      Serial.println("2");
      delay(1000);
      Serial.println("1");
      delay(1000);
      digitalWrite(charge, LOW);
      delay(switchDelay);
      digitalWrite(kick, HIGH);
      delay(pulseTime);
      digitalWrite(kick, LOW);
      delay(switchDelay);
      //digitalWrite(charge, HIGH);
      delay(200);
    }
  }
}