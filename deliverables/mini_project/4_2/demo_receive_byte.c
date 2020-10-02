#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int number = 0;

void setup() {
  // start serial for output
  // Serial.begin(9600); <-For debugging
  
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
}

// idle when inactive
void loop() {
  delay(10);
}

// callback for received data
void receiveData(int byteCount){

  while(Wire.available()) {
    // receive number sent to arduino
    number = Wire.read();
  }
}
