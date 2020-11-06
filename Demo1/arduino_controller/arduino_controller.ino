/**
 * This is the "main" file for the Demo1 controller, which contains the setup()
 * and loop() for the actual controller itself.
 *
 * @author Luke Henke
 * @authur Gabe Alcantar-Lopez
 *
 * Class: Seed Lab.
 */
#include "constants.h"
#include <Encoder.h>
#include "functions.h"
#include "phi_controller.h"
#include "rho_controller.h"

// -------- I2C defines --------
#define SLAVE_ADDRESS 0x04

// Setup the wheel encoder.
Encoder rightWheel1(A_1_PIN, B_1_PIN); // right wheel is wheel #1
Encoder leftWheel2(A_2_PIN, B_2_PIN);

void setup() {
    pinMode(MOTOR_1_PWM, OUTPUT);
    pinMode(MOTOR_1_DIRECTION, OUTPUT);
    pinMode(MOTOR_2_PWM, OUTPUT);
    pinMode(MOTOR_2_DIRECTION, OUTPUT);

    // Turn on the ENABLE pin
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);

    // Setup SF / status. LOW is a fault.
    pinMode(STATUS_FLAG, INPUT);

    Serial.begin(74880);
}

void loop() {
    unsigned long start_time = millis();

    static double oldWheel1RPosition = 0;
    static double oldWheel2LPosition = 0;

    // Get the theta's
    double newWheel1RPosition = rightWheel1.read() * 2*PI / COUNTS_PER_ROTATION; // [rad]
    double newWheel2LPosition = -1*leftWheel2.read() * 2*PI / COUNTS_PER_ROTATION; // [rad]

    // Get the theta_dots.
    double rightWheel1Velocity = (newWheel1RPosition - oldWheel1RPosition) / PERIOD * 1e3; // [rad/s]
    double leftWheel2Velocity = (newWheel2LPosition - oldWheel2LPosition) / PERIOD * 1e3; // [rad/s]

    // rho/phi_dot are based on the velocity measurements
    double rho_dot = WHEEL_RADIUS * (rightWheel1Velocity + leftWheel2Velocity) / 2;
    double phi_dot = WHEEL_RADIUS * (rightWheel1Velocity - leftWheel2Velocity) / DISTANCE_B_W_WHEELS;

    // phi/rho (normal) are calculated based on position measurements
    double phi = WHEEL_RADIUS * (newWheel1RPosition - newWheel2LPosition) / DISTANCE_B_W_WHEELS;
    double rho = WHEEL_RADIUS * (newWheel1RPosition + newWheel2LPosition) / 2;

    // If this is demo2, do the turn first, *then* move the 1 foot
    #ifdef DEMO_2
        SET_RHO = 0.0;
        // Only do the PHI controller until we've basically gotten it right
        if (withinEpsilon(phi, SET_PHI, 0.02)) { // withinEpsilon(phi, SET_PHI, 0.01)
            // Then, turn on the RHO controller
            SET_RHO = 0.3048; // 1 foot in meters
        }
    #endif

    // Get the new values from the control system
    double deltaVa = completePhiController(phi_dot, phi);
    double Va = completeRhoController(rho_dot, rho);

    // If both constraints are satisified, shut off the controller
    if (withinEpsilon(phi, SET_PHI, 0.005) && withinEpsilon(rho, SET_RHO, 0.02)) {
        deltaVa = Va = 0;
    }

    // If the VAs are small, then just round them down to 0.
    if (abs(deltaVa) <= 0.3) deltaVa = 0;
    if (abs(Va) <= 0.3) Va = 0;

    // Get wheel voltages from these
    double rightWheel1Voltage = calculateV1(Va, deltaVa);
    double leftWheel2Voltage = calculateV2(Va, deltaVa);

    // Convert to PWM & direction
    double rightWheel1Direction = WHEEL_1_FORWARD, leftWheel2Direction = WHEEL_2_FORWARD;
    if (rightWheel1Voltage < 0) {
        rightWheel1Voltage *= -1;
        rightWheel1Direction = WHEEL_1_BACKWARD;
    }

    if (leftWheel2Voltage < 0) {
        leftWheel2Voltage *= -1;
        leftWheel2Direction = WHEEL_2_BACKWARD;
    }

    double rightWheel1PWM = voltageToPWM(rightWheel1Voltage);
    double leftWheel2PWM = voltageToPWM(leftWheel2Voltage);

    // Set the values
    analogWrite(MOTOR_1_PWM, rightWheel1PWM);
    digitalWrite(MOTOR_1_DIRECTION, rightWheel1Direction);

    analogWrite(MOTOR_2_PWM, leftWheel2PWM);
    digitalWrite(MOTOR_2_DIRECTION, leftWheel2Direction);

    #ifdef DEBUG_MAIN
        // Print values of interest
        // Serial.print(start_time);
        Serial.print("\t");
        Serial.print(rightWheel1Velocity);
        Serial.print("\t");
        Serial.print(leftWheel2Velocity);
        Serial.print("\t");
        // Serial.print(rho_dot);
        // Serial.print("\t");
        // Serial.print(phi_dot);
        // Serial.print("\t");
        Serial.print(rho);
        Serial.print("\t");
        Serial.print(phi);
        Serial.print("\t");
        Serial.print(deltaVa);
        Serial.print("\t");
        // Serial.print(Va);

        // Serial.print(rightWheel1PWM);
        // Serial.print("\t");
        // Serial.print(leftWheel2PWM);
        // Serial.println();
        // Serial.print(start_time);
        // Serial.print("\t");
        // Serial.print(SET_PHI);
        // Serial.print("\t");
        // Serial.print(phi);
        Serial.println();
    #endif

    oldWheel1RPosition = newWheel1RPosition;
    oldWheel2LPosition = newWheel2LPosition;

    unsigned long end_time = millis();

    if (end_time - start_time > PERIOD) {
        Serial.println("ERROR - Main takes too long");
    }

    // Wait for PERIOD time
    while (millis() < start_time + PERIOD);
}
