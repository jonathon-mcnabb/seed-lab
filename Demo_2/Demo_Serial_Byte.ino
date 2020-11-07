void setup() {
  // start serial for output
  Serial.begin(115200);
}

// wait for the pi to send a number and return the number plus 5
void loop() {
  if (Serial.available() > 0) {
    int data = Serial.read() + 5;
    Serial.println(data);
  }
}
