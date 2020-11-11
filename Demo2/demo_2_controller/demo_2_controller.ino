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
#include <Wire.h>

#include "functions.h"
#include "phi_controller.h"
#include "rho_controller.h"
#include "points_buffer.h"

#define DEMO_2

// #define WIRE_PIN 8

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
    Serial.setTimeout(2);
    Wire.begin(0x8);
    Wire.onReceive(fromI2C);


    // negative angle = left spin
    // positive angle = right spin
    // pointQueue.enqueue({0.6096, PI/2});
    // pointQueue.enqueue({1.2192, -PI/2});
    // pointQueue.enqueue({1.2192, -PI/2});
    // pointQueue.enqueue({1.2192, -PI/2});
    // pointQueue.enqueue({0.6096, -PI/2});
}

double angleFromPi = 0.0;
double distanceFromMarker = 0.0;
bool receivedAngleFromPi = false;
bool receivedDistanceFromPi = false;
void fromI2C(const size_t byteCount) {
    static char buffer[64];
    static size_t i = 0;
    static boolean seenAnAorD = false;
    boolean parseNumber = false;

    while (Wire.available()) {
        char c = Wire.read();
        Serial.print(c);
        if ((c != 'A' && c != 'D') && !seenAnAorD)
            continue;

        seenAnAorD = true;
        if (c == 'S') {
            seenAnAorD = false;
            parseNumber = true;
            break;
        }


        buffer[i] = c;
        i++;
        i %= 64;
    }

    // Once we've read 7 bytes, we've gotten the angle
    // If we have _all_ data from the PI, then it's seen the marker
    // We *always* expect: A<angle>
    //      where <angle> is of the form: X.XXXX (radians)
    // when in this state
    // this at least 7 bytes long
    // easily long enough to be in the buffer
    if (parseNumber) {
        Serial.println();
        buffer[i] = '\0'; // null-terminate the string
        i++;

        // Serial.print("Raw buffer: ");
        // Serial.println(buffer);

        double parsedDouble = strtod(buffer+1, nullptr); // Skip over the 'A' or 'D'

        switch (buffer[0]) {
            case 'A':
                receivedAngleFromPi = true;
                angleFromPi = parsedDouble;
                break;

            case 'D':
                receivedDistanceFromPi = true;
                distanceFromMarker = parsedDouble;
                break;
        }

        i = 0;

        // Serial.print("Parse double from I2C: ");
        Serial.println(parsedDouble);
        parseNumber = false;
    }
}

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

    // Serial.print("\t");
    // Serial.print(rightWheelVoltage);
    // Serial.print("\t");
    // Serial.print(leftWheelVoltage);
    // Serial.print("\t");
    // Serial.println();

    analogWrite(MOTOR_1_PWM, rightWheelPWM);
    digitalWrite(MOTOR_1_DIRECTION, rightWheelDirection);
    analogWrite(MOTOR_2_PWM, leftWheelPWM);
    digitalWrite(MOTOR_2_DIRECTION, leftWheelDirection);
}

double turnController(const double currentAngle, const double setAngle) {
    const double Kp = 15;
    const double Ki = 220;
    static double sum = 0;
    const double fromPController = pController(currentAngle, setAngle, Kp);
    const double fromIController = iController(currentAngle, setAngle, sum, Ki);

    return fromPController + fromIController;
}

typedef enum { SPIN_TO_ARUCO, MOVE_TO_ARUCO, UPDATE_SET_POINT, SPIN_TO_SET_ANGLE, MOVE_TO_SET_RADIUS, HALT_ROBOT } fsm_t;
void loop() {
    const unsigned long start_time = millis();
    static fsm_t state = SPIN_TO_ARUCO; // Default state: spin until you see the aruco marker

    static double oldAngle = 0;
    static double oldRadius = 0;
    static double oldX = 0;
    static double oldY = 0;

    static double piSetRadius = 0;
    static double piSetAngle = 0;

    // -------------------
    // MARK: Measure all variables
    // -------------------
    const double rightWheelPosition = rightWheel1.read() * 2*PI / COUNTS_PER_ROTATION;
    const double leftWheelPosition = -1*leftWheel2.read() * 2*PI / COUNTS_PER_ROTATION;

    // Since the two positions above are in radians, we can convert to polar
    const double currentAngle = (WHEEL_RADIUS * (rightWheelPosition - leftWheelPosition) / DISTANCE_B_W_WHEELS);
    const double currentRadius = WHEEL_RADIUS * (rightWheelPosition + leftWheelPosition) / 2;

    #ifdef DEBUG_MAIN
        Serial.print("\t");
        Serial.print(currentAngle);
        Serial.print("\t");
        Serial.print(currentRadius);
        Serial.print("\t");
    #endif

    // Then state machine this
    switch (state) {
        case SPIN_TO_ARUCO: {
            // Assume spin 180-degrees until the PI sends us data
            // and once it does, remember the old data
            static double setAngle = 2*PI;
            static boolean turnOnController = false;
            static boolean readAnAngleBefore = false;
            static boolean turnOffMotor = false;

            // TODO: Patch up this logic. Cause this sucks...
            if (receivedAngleFromPi && !readAnAngleBefore) {
                receivedAngleFromPi = false;

                Serial.print("Current angle: ");
                Serial.println(currentAngle);
                Serial.print("Set angle: ");
                Serial.println(setAngle);

                turnOffMotor = true;
                readAnAngleBefore = true;
            }

            if (receivedAngleFromPi && readAnAngleBefore && !turnOnController) {
                static double previousMeasurement = 10.0;
                receivedAngleFromPi = false;

                if (withinEpsilon(previousMeasurement, angleFromPi, 0.005)) {
                    setAngle = angleFromPi + (angleFromPi < 0 ? 0.1 : -0.05);
                    // currentAngle = 0;
                    turnOnController = true;
                    turnOffMotor = false;
                } else {
                    receivedAngleFromPi = false;
                    previousMeasurement = angleFromPi;
                    turnOffMotor = true;
                    rightWheel1.write(0);
                    leftWheel2.write(0);
                }
            }

            // If we're "close enough", go to UPDATE_SET_POINT
            if (withinEpsilon(setAngle, currentAngle, 0.01)) {
                setMotorsVoltage(0, 0);
                state = MOVE_TO_ARUCO;
                break;
            }

            // if (withinEpsilon(angleFromPi, 0.0, 0.01)) {
            //     state = HALT_ROBOT;
            //     break;
            // }

            // Spin us to the set angle
            // const double Kp = 5;
            Serial.print("Current angle: ");
            Serial.println(currentAngle);
            Serial.print("Set angle: ");
            Serial.println(setAngle);

            const double deltaVa = (turnOnController ? turnController(currentAngle, setAngle) : 4.0);
            // const double deltaVa = turnOnController ?
                // pController(currentAngle, setAngle, 15.0)
                // : 2;
            const double Va = 0;

            if (turnOffMotor) {
                setMotorsVoltage(0, 0);
            } else {
                setMotorsVoltage(Va, deltaVa);
            }
            break;
        }

        // Moves to within 1 foot of the aruco marker
        case MOVE_TO_ARUCO: {
            const double distanceToMove = distanceFromMarker - 0.1;
            Serial.print("Move to: ");
            Serial.println(distanceToMove);

            // Enqueue the distance
            pointQueue.enqueue({ distanceToMove, 0 });

            // Enqueue all the points
            #ifdef DEMO_2
                pointQueue.enqueue({0.3448, PI/2}); // move right
                pointQueue.enqueue({0.6496, -PI/2}); // move up
                pointQueue.enqueue({0.8096, -PI/2}); // move left
                pointQueue.enqueue({0.7096, -PI/2}); // move down
                pointQueue.enqueue({0.3548, -PI/2}); // move right
            #endif

            state = UPDATE_SET_POINT;
            break;
        }

        /**
         * Grabs the top/front element from the queue.
         */
        case UPDATE_SET_POINT: {
            // If we've ran out of points, jump to halt.
            if (!has_another_point()) {
                state = HALT_ROBOT;
                break;
            }

            point new_point = get_next_point();
            piSetRadius = new_point.radius;
            piSetAngle = new_point.angle;

            state = SPIN_TO_SET_ANGLE;
            rightWheel1.write(0);
            leftWheel2.write(0);
            break;
        }

        /**
         * Only spin towards the robot, with no lateral movement
         * The angle we should aim towards is based on currentX/Y and the setX/Y
         */
        case SPIN_TO_SET_ANGLE: {
            const double setAngle = piSetAngle; //atan2(deltaY, deltaX); // returns angle in radians

            Serial.print("Facing set angle\t");

            // If we are "close enough" to the actual angle we need to face, then go to MOVE state
            if (withinEpsilon(setAngle, currentAngle, 0.02)) {
                state = MOVE_TO_SET_RADIUS;
                setMotorsVoltage(0, 0); // Turn off motors on transition to prevent over-turn.
                delay(100);
                break;
            }

            // Else, turn until we get there.
            // const double Kp = 15;
            // const double deltaVa = pController(currentAngle, setAngle, Kp);
            const double deltaVa = turnController(currentAngle, setAngle);
            const double Va = 0;

            setMotorsVoltage(Va, deltaVa);
            break;
        }

        /**
         * State MOVE_TO_SET_RADIUS: move until we've moved the correct radius
         *
         * We always keep the wheels moving forward at a nominal rate, say 2 V, and then we control Kp
         * such that we are always pointing toward the set point.
         */
        case MOVE_TO_SET_RADIUS: {
            // Hold the angle steady, while we move steadily forward _to_ the set radiuan
            const double setAngle = piSetAngle;

            Serial.print("Moving to RADIUS\t");

            // If we get close enough, head to the next point.
            if (withinEpsilon(currentRadius, piSetRadius, 0.01)) {
                setMotorsVoltage(0, 0);
                state = UPDATE_SET_POINT;
                rightWheel1.write(0);
                leftWheel2.write(0);
                delay(100);
                break;
            }

            // Use a pController to change the angle
            const double Kp = 15;
            const double deltaVa = pController(currentAngle, setAngle, Kp);
            // const double deltaVa = turnController(currentAngle, setAngle);
            const double Va = NOMINAL_VOLTAGE;
            setMotorsVoltage(Va, deltaVa);
            break;
        }

        /**
         * When we've hit halt, shut off.
         */
        case HALT_ROBOT: {
            setMotorsVoltage(0, 0);
            break;
        }

        default:
            // How did you get here?
            Serial.print("ISR state hit default?\r\n");
            break;
    }

    #ifdef DEBUG_MAIN
        Serial.println();
    #endif

    const unsigned long end_time = millis();

    if (end_time - start_time > PERIOD) {
        Serial.print("ERROR - Main takes too long: \t");
        Serial.println(end_time - start_time);
    }

    // Wait for PERIOD time
    while (millis() < start_time + PERIOD);
}
