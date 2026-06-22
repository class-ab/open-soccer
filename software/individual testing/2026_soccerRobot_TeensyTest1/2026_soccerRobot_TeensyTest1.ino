int button1 = A6;
int button2 = A7;
int button3 = A8;

void setup() {
  Serial.begin(9600);
  pinMode(button1, INPUT);
  pinMode(button1, INPUT);
  pinMode(button1, INPUT);
}

void loop() {
  if (digitalRead(button1) == HIGH) {
    Serial.println("1");
    delay(200);
  }
  if (digitalRead(button2) == HIGH) {
    Serial.println("2");
    delay(200);
  }
  if (digitalRead(button3) == HIGH) {
    Serial.println("3");
    delay(200);
  }
}
