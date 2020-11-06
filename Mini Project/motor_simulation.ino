/**
 * This file contains the open-loop step response on the motor
 * for use in MATLAB for the system-model of the wheel, so that
 * the control system could be designed.
 *
 * Additionally, this file contains the skeleton code required
 * to ensure main() runs within a specified amount of time
 *
 * This uses the Encoder.h library
 *
 * @author Luke Henke
 * @author Gabe Alcantar-Lopez
 *
 * Class: Seed Lab
 */

//Define constants and import Encoder.h library
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#define PERIOD 5
#define encoderOneAPin 2
#define encoderOneBPin 3
#define MOTOR_ONE_PWM 9
#define ENABLE_PIN 4
#define MOTOR_ONE_DIRECTION 7
#define COUNTS_PER_ROTATION 64

//Set encoder pins
Encoder wheel(encoderOneAPin, encoderOneBPin);
int motorVoltage = 0;



void setup() {
  Serial.begin(74880);
  //Set motor PWM and Motor direction pins as outputs
  pinMode(MOTOR_ONE_PWM, OUTPUT);
  pinMode(MOTOR_ONE_DIRECTION, OUTPUT);
  //Assign PWM as analog value from Arduino
  analogWrite(MOTOR_ONE_PWM, motorVoltage);
  //Assign direction as digital input from Arduino
  digitalWrite(MOTOR_ONE_DIRECTION, 1);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, 1);


}

void loop() {
    //Variable to read current time
    unsigned long time_now = millis();
    static bool startedStep = false;

    //IF statement to ensure 1 second has elapsed before starting motor
    if(time_now > 1000 && startedStep == false){
        //Set motor voltage to full value
        motorVoltage = 255;
        analogWrite(MOTOR_ONE_PWM, motorVoltage);
        startedStep = true;

    }

    //IF statement to ensure data is read between 1 and 2 second
    if (time_now > 1000 && time_now < 2000){
        static double oldPosition = 0;
        double newPosition = wheel.read()*2*PI / COUNTS_PER_ROTATION;

        double velocity = (newPosition - oldPosition) / PERIOD * 1e3;

        //Print values of interest
        Serial.print(time_now);
        Serial.print("\t");
        Serial.print(motorVoltage);
        Serial.print("\t");
        Serial.print(velocity);
        Serial.print("\t");
        Serial.print(wheel.read());
        Serial.println();
        oldPosition = newPosition;
    }

    unsigned long currentTime = millis();
    if( currentTime - time_now > PERIOD){
      Serial.println("ERROR - Main takes too long");
    }

    while(millis() < time_now + PERIOD){
        //wait approx. [period] ms
    }
}
