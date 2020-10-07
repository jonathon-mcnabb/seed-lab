/**
 * This file performs the closed loop step response of the controller, live on the wheel.
 * @author Luke Henke
 * @authur Gabe Alcantar-Lopez
 *
 * Class: Seed Lab.
 */
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

// A & B pins for encoder 1
#define A_1_PIN 2
#define B_1_PIN 3

#define ANGULAR_RESET_PIN 8

// Constants used in the calculations
#define COUNTS_PER_ROTATION (64*50) // 64 if both A&B on interrupt pins, 16 else

// -------- Motor Defines ----------
#define MOTOR_1_PWM 9
#define MOTOR_1_DIRECTION 7

#define ENABLE_PIN 4 // for turning off & on the motor
#define STATUS_FLAG 12

#define CLOCKWISE HIGH
#define COUNTERCLOCKWISE LOW

// -------- I2C defines --------
#define SLAVE_ADDRESS 0x04

// Setup the wheel encoder.
Encoder wheel(A_1_PIN, B_1_PIN);

void setup() {
    pinMode(MOTOR_1_PWM, OUTPUT);
    pinMode(MOTOR_1_DIRECTION, OUTPUT);

    // Turn on the ENABLE pin
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);

    // Setup SF / status. LOW is a fault.
    pinMode(STATUS_FLAG, INPUT);

    Serial.begin(74880);
}

#define PERIOD 7 // in MS
#define BATTERY_MAX_VOLTAGE 7.3

static double setPosition = 0;

/**
 * Ensures x doesn't get smaller than minVal, or larger than maxVal
 * @param  x
 * @param  minVal
 * @param  maxVal
 * @return
 */
double mapToRange(double x, double minVal, double maxVal) {
    if (x > maxVal) {
        return maxVal;
    }

    if (x < minVal) {
        return minVal;
    }

    return x;
}

/**
 * Runs our PID controller
 * @param  new_position the position in RADIANS
 * @return              Vout
 */
double controller(double newPosition) {
    // Units out of matlab were in terms of seconds, we want MS.
    static const double Kp = 0.00536*1e3;
    static const double Kd = 0.01509*1e3;
    static const double Ki = 0.00043*1e3;

    // static double oldPosition = 0; // TODO: provide a way to set this to 0 while the program is running
    static double oldError = 0;
    static double sum = 0;

    double newError = setPosition - newPosition;
    sum += newPosition;

    double pTerm = (newError) * Kp;
    double dTerm = (newError - oldError) * Kd / PERIOD;
    double iTerm = sum * PERIOD * Ki; //sum * PERIOD * Ki;
    oldError = newError;

    double Vout = pTerm + iTerm + dTerm;
    return mapToRange(Vout, -BATTERY_MAX_VOLTAGE, BATTERY_MAX_VOLTAGE);
}

void loop() {
    unsigned long start_time = millis();

    // Grab the current position
    double newPosition = wheel.read() * 2*PI / (COUNTS_PER_ROTATION);

    // Get the new voltage from our controller
    double Vout = controller(newPosition);

    // If the Vout is negative
    // spin the motor in the other direction
    int direction = CLOCKWISE;
    if (Vout < 0) {
        Vout *= -1; // make Vout positive.
        direction = COUNTERCLOCKWISE;
    }

    int pwm = (Vout * 255 / BATTERY_MAX_VOLTAGE);

    // Set the PWM
    analogWrite(MOTOR_1_PWM, pwm);
    digitalWrite(MOTOR_1_DIRECTION, direction);


    if (start_time > 1000 && start_time < 2000){
        setPosition = 4 * PI;

    }

    static double oldPosition = 0;
    double velocity = (newPosition - oldPosition) / PERIOD * 1e3;
    //Print values of interest
    Serial.print(start_time);
    Serial.print("\t");
    Serial.print(pwm);
    Serial.print("\t");
    Serial.print(velocity);
    Serial.print("\t");
    Serial.print(newPosition);
    Serial.println();
    oldPosition = newPosition;

    unsigned long end_time = millis();

    if (end_time - start_time > PERIOD) {
        Serial.println("ERROR - Main takes too long");
    }

    // Wait for PERIOD time
    while (millis() < start_time + PERIOD);
}
