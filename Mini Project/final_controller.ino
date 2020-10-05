#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Wire.h> // for I2C comms

// Uncomment these # defines to add some debugging output.
// #define DEBUG_ISR
// #define I2C_DEBUG
// #define DEBUG_MAIN

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

    // Setup I2C comms
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(dataFromI2C);
}

#define PERIOD 7 // in MS
#define BATTERY_MAX_VOLTAGE 7.3

static double setPosition = 0;
void dataFromI2C(int numBytes) {
    // For right now, we're only getting 1 byte at a time.
    byte quadrant = Wire.read();

    #ifdef I2C_DEBUG
        Serial.print("From I2C: ");
        Serial.print(quadrant);
        Serial.println();
    #endif

    setPosition = quadrant * PI / 2;
}

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
 * Runs our PID controller, which is really just a P controller
 * @param  new_position the position in RADIANS
 * @return              Vout
 */
double controller(double newPosition) {
    static const double Kp = 10.0;
    static const double Kd = 0;
    static const double Ki = 0;

    // static double oldError = 0;
    // static double sum = 0;

    double newError = setPosition - newPosition;
    // sum += newError;

    double pTerm = (newError) * Kp;
    // double dTerm = (newError - oldError) * Kd / PERIOD;
    // double iTerm = sum * PERIOD * Ki;

    #ifdef DEBUG_ISR
        static long i = 0;
        i++;

        if (i%100 == 0) {

            Serial.print(setPosition);
            Serial.print("\t");
            Serial.print(newPosition);
            Serial.print("\t");
            Serial.print(newError);
            Serial.print("\t");
            Serial.println(pTerm);
        }
    #endif

    // oldError = newError;
    // oldPosition = newPosition;

    double Vout = pTerm;
    return mapToRange(Vout, -BATTERY_MAX_VOLTAGE, BATTERY_MAX_VOLTAGE);
}

void loop() {
    unsigned long start_time = millis();

    // Grab the current position
    double newPosition = wheel.read() * 2*PI / (COUNTS_PER_ROTATION);

    // Get the new voltage from our controller
    double Vout = controller(newPosition);

    // If the Vout is negative, tell the motor to spin the motor in the other direction
    int direction = CLOCKWISE;
    if (Vout < 0) {
        Vout *= -1; // make Vout positive.
        direction = COUNTERCLOCKWISE;
    }

    int pwm = (Vout * 255 / BATTERY_MAX_VOLTAGE);

    // Set the PWM
    analogWrite(MOTOR_1_PWM, pwm);
    digitalWrite(MOTOR_1_DIRECTION, direction);

    #ifdef DEBUG_MAIN
        static long i = 0;
        i++;

        // Only print once every 100 times.
        if (i % 100 == 0) {
            Serial.print(newPosition);
            Serial.print("\t");
            Serial.print(wheel.read());
            Serial.print("\t");
            Serial.print(Vout);
            Serial.print("\t");
            Serial.print(pwm);
            Serial.println();
        }
    #endif

    unsigned long end_time = millis();

    if (end_time - start_time > PERIOD) {
        Serial.println("ERROR - Main takes too long");
    }

    // Wait for PERIOD time
    while (millis() < start_time + PERIOD);
}
