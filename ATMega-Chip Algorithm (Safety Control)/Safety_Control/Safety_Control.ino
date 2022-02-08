int step_pin = 3;
int dir_pin = 4;
int solid1 = 5;
int solid2 = 7;
int solid3 = 6;
int Tiva = 8;
int LED = 9;
int LED_state = 10;

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(step_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  pinMode(solid1, OUTPUT);
  pinMode(solid2, OUTPUT);
  pinMode(solid3, OUTPUT);
  pinMode(Tiva, OUTPUT);
  pinMode(LED_state, INPUT_PULLUP);
  digitalWrite(dir_pin, HIGH);
  
  delay(2000);
  digitalWrite(solid1, HIGH);

  delay(2000);
  digitalWrite(solid2, HIGH);

  delay(2000);
  digitalWrite(solid3, HIGH);
  
  delay(2000);
  digitalWrite(Tiva, HIGH);
}

void loop() {
  if (digitalRead(LED_state) == HIGH) {
    analogWrite(LED,255);//255
  } else {
    analogWrite(LED, 150);//150
  }
  digitalWrite(step_pin, HIGH);
  delayMicroseconds(90);
  
  digitalWrite(step_pin, LOW);
  delayMicroseconds(3750 / 16);

}
