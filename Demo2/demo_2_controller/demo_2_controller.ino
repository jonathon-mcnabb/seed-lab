/**
 * This file contains the control system for Demo 2.
 *
 * @author Luke Henke
 * @authur Gabe Alcantar-Lopez
 *
 * Class: Seed Lab.
 */
#include "constants.h"

#include <Encoder.h>
#include <ArduinoQueue.h> // for using a queue.
#include <math.h>

#include "functions.h"
#include "phi_controller.h"
#include "rho_controller.h"
#include "points_buffer.h"

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

    Serial.begin(115200);
}


// Make 2 functions:
// 1 - find aruco marker (spin slowly until angle of 0), read distance, move to within 6 inches
// 2 - "spin in circle". Can also do a box.

/*
Or, do Darren's idea: keep track of current (x, y), and have a set (x, y)

State machine: {SPIN_TO_ARUCO, MOVE_TO_X_Y, UPDATE_X_Y, SHUT_OFF }

case SPIN_TO_ARUCO:
    * spin slowly until you see the aruco marker,
    * and then keep spinning until the controller is angle 0
    * the robot is now at (0, 0)

case UPDATE_X_Y:
    * pop set point off of queue, this becomes the set X_Y

case MOVE_TO_X_Y:
    * have the wheels move at a nominal rate, say 2 V or 4 V
    * measure current (x, y)
    * measure the current angle
    * update the set angle, arctan (delta Y / delta X) I think?
    * have a Kp controller on angle (just a phi controller)
    *
    *  deltaVa = phiController(set angle, current angle)
    *
    * leftWheel = 4 V + deltaVaToV1()
    * rightWheel = 4 V + deltaVaToV2()



 */

/**
 * Sets the voltage to the motors based on Va and deltaVa
 * @param Va      forward movement
 * @param deltaVa rotational movement
 */
void setMotorsVoltage(double Va, double deltaVa) {
    const double rightWheelVoltage = boundValue(calculateV1(Va, deltaVa), BATTERY_MAX_VOLTAGE, -1*BATTERY_MAX_VOLTAGE);
    const double leftWheelVoltage = boundValue(calculateV2(Va, deltaVa), BATTERY_MAX_VOLTAGE, -1*BATTERY_MAX_VOLTAGE);

    const double rightWheelDirection = (rightWheelVoltage >= 0) ? WHEEL_1_FORWARD : WHEEL_1_BACKWARD;
    const double leftWheelDirection = (leftWheelVoltage >= 0) ? WHEEL_2_FORWARD : WHEEL_2_BACKWARD;
    const double rightWheelPWM = voltageToPWM(rightWheelVoltage);
    const double leftWheelPWM = voltageToPWM(leftWheelVoltage);

    Serial.print("\t");
    Serial.print(rightWheelVoltage);
    Serial.print("\t");
    Serial.print(leftWheelVoltage);
    Serial.print("\t");
    Serial.println();

    analogWrite(MOTOR_1_PWM, rightWheelPWM);
    digitalWrite(MOTOR_1_DIRECTION, rightWheelDirection);
    analogWrite(MOTOR_2_PWM, leftWheelPWM);
    digitalWrite(MOTOR_2_DIRECTION, leftWheelDirection);

    pointQueue.enqueue({0, 0});
}

typedef enum { SPIN_TO_ARUCO, UPDATE_X_Y, FACE_X_Y, MOVE_TO_X_Y, HALT_ROBOT } fsm_t;
void loop() {
    const unsigned long start_time = millis();
    static fsm_t state = SPIN_TO_ARUCO; // Default state: spin until you see the aruco marker

    static double oldAngle = 0;
    static double oldRadius = 0;
    static double oldX = 0;
    static double oldY = 0;

    static double setX = 0.9144;
    static double setY = 0.9144;

    // -------------------
    // MARK: Measure all variables
    // -------------------
    const double rightWheelPosition = rightWheel1.read() * 2*PI / COUNTS_PER_ROTATION;
    const double leftWheelPosition = -1*leftWheel2.read() * 2*PI / COUNTS_PER_ROTATION;

    // Since the two positions above are in radians, we can convert to polar
    const double overallAngle = (WHEEL_RADIUS * (rightWheelPosition - leftWheelPosition) / DISTANCE_B_W_WHEELS);
    const double deltaAngle = overallAngle - oldAngle;
    const double overallRadius = WHEEL_RADIUS * (rightWheelPosition + leftWheelPosition) / 2;
    const double deltaRadius = overallRadius - oldRadius;

    // And then convert from polar to cartesian
    // If the angle hasn't changed, then we can just do rho*cos(theta)
    // If it has, then we've done some differential equations work to get good values.
    const double currentX = (deltaAngle == 0) ?
        // deltaRadius * cos(oldAngle) + oldX
        overallRadius * cos(overallAngle)
         : (deltaRadius / deltaAngle) * (sin(overallAngle) - sin(oldAngle)) + oldX;
    // const double currentX = overallRadius * cos(overallAngle);
    // const double currentY = overallRadius * sin(overallAngle);

    const double currentY = (deltaAngle == 0) ?
        // deltaRadius * sin(oldAngle) + oldY
        overallRadius * sin(overallAngle)
         : (deltaRadius / deltaAngle) * (cos(oldAngle) - cos(overallAngle)) + oldY;

    // Find the set angle
    const double deltaX = setX - currentX;
    const double deltaY = setY - currentY;

    // Then state machine this
    switch (state) {
        case SPIN_TO_ARUCO: {
            // Assume spin 180-degrees until the PI sends us data
            // and once it does, remember the old data
            static double setAngle = PI;

            // If we have _all_ data from the PI, then it's seen the marker
            // We *always* expect: A<angle>
            //      where <angle> is of the form: X.XXXX (radians)
            // when in this state
            // this at least 7 bytes long
            // easily long enough to be in the buffer
            if (Serial.available() >= 7) {
                // Verify its of the form we expected
                if (Serial.peek() != 'A') {
                    Serial.print("Unexpected input character: ");
                    Serial.println(Serial.peek());
                } else {
                    const double angleFromSerial = Serial.parseFloat();
                    Serial.print("Parse angle from serial: ");
                    Serial.println(angleFromSerial);

                    // If we're "close enough", go to UPDATE_X_Y
                    if (withinEpsilon(angleFromSerial, 0.0, 0.01)) {
                        state = UPDATE_X_Y;
                        break;
                    }

                    // TODO: verify this math
                    setAngle = overallAngle + angleFromSerial;
                }
            }

            // Spin us to the set angle
            const double Kp = 5;
            const double deltaVa = pController(overallAngle, setAngle, Kp);
            const double Va = 0;

            setMotorsVoltage(Va, deltaVa);
            break;
        }

        /**
         * Grabs the top/front element from the queue.
         */
        case UPDATE_X_Y: {
            // If we've ran out of points, jump to halt.
            if (!has_another_point()) {
                state = HALT_ROBOT;
                break;
            }

            point new_point = get_next_point();
            setX = new_point.x;
            setY = new_point.y;

            state = FACE_X_Y;
            break;
        }

        /**
         * Only spin towards the robot, with no lateral movement
         * The angle we should aim towards is based on currentX/Y and the setX/Y
         */
        case FACE_X_Y: {
            const double setAngle = atan2(deltaY, deltaX); // returns angle in radians

            Serial.print(setAngle);
            Serial.print("\t");
            Serial.print(overallAngle);
            // Serial.println();

            // If we are "close enough" to the actual angle we need to face, then go to MOVE state
            if (withinEpsilon(setAngle, overallAngle, 0.05)) {
                state = MOVE_TO_X_Y;
                // state = HALT_ROBOT;
                break;
            }

            // Else, turn until we get there.
            const double Kp = 15;
            const double deltaVa = pController(overallAngle, setAngle, Kp);
            const double Va = 0;

            setMotorsVoltage(Va, deltaVa);
            break;
        }

        /**
         * State MOVE_TO_X_Y: move until we get to X/Y
         *
         * We always keep the wheels moving forward at a nominal rate, say 2 V, and then we control Kp
         * such that we are always pointing toward the set point.
         *
         * Current angle: measure from encoders
         * Current x,y: ??
         * Set angle: arctan(deltaY / deltaX) ??
         */
        case MOVE_TO_X_Y: {
            // Hold the angle steady, while we move steadily forward _to_ the (X, Y)
            const double setAngle = atan2(deltaY, deltaX); // returns angle in radians
            Serial.print(setX);
            Serial.print("\t");
            Serial.print(setY);
            Serial.print("\t");
            Serial.print(currentX);
            Serial.print("\t");
            Serial.print(currentY);
            Serial.print("\t");
            Serial.print(setAngle);
            // Serial.println();

            // If we get close enough, head to the next point.
            if (withinEpsilon(deltaX, 0.0, 0.05) && withinEpsilon(deltaY, 0.0, 0.05)) {
                state = UPDATE_X_Y;
                break;
            }

            // Use a pController to change the angle
            const double Kp = 15;
            // const double deltaVa = 0;
            const double deltaVa = pController(overallAngle, setAngle, Kp);
            const double Va = NOMINAL_VOLTAGE;
            setMotorsVoltage(Va, deltaVa);
            break;
        }

        /**
         * When we've hit halt, shut off.
         */
        case HALT_ROBOT: {
            analogWrite(MOTOR_1_PWM, 0);
            analogWrite(MOTOR_2_PWM, 0);
            break;
        }

        default:
            // How did you get here?
            Serial.print("ISR state hit default?\r\n");
            break;
    }


    // Update old variables
    oldAngle = overallAngle;
    oldRadius = overallRadius;
    oldX = currentX;
    oldY = currentY;

    const unsigned long end_time = millis();

    if (end_time - start_time > PERIOD) {
        Serial.println("ERROR - Main takes too long");
    }

    // Wait for PERIOD time
    while (millis() < start_time + PERIOD);
}
