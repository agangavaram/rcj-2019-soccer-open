void setup() {
  Serial.begin(9600);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
}

void loop() {
  Serial.print("Left: ");
  Serial.println(digitalRead(2));
  Serial.print("Right: ");
  Serial.println(digitalRead(3));

}
