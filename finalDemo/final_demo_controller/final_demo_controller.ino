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
#include "points_buffer.h"

// Comment out this line of code to only do the first-half of Demo2, i.e. rotate and move to within a foot.
#define DEMO_2

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

    // Setup I2C comms
    Wire.begin(0x8);
    Wire.onReceive(fromI2C);

    // Move forward 1 foot
    // add_point_to_buffer(0.3048, PI/2);
    pointQueue.enqueue({0.3448, PI/2}); // move right
    pointQueue.enqueue({0.7096, -PI/2}); // move up
    pointQueue.enqueue({0.8096, -PI/2}); // move left
    pointQueue.enqueue({0.7096, -PI/2}); // move down
    pointQueue.enqueue({0.3548, -PI/2}); // move right
}


// These global variables, along with the function below, work to handle
// all communication from the PI.
//
// Based on what is received, the appropriate value is set, and the appropriate
// boolean flag is set to true.
double angleFromPi = 0.0;
double distanceFromMarker = 0.0;
bool receivedAngleFromPi = false;
bool receivedDistanceFromPi = false;
double turnControllerSum = 0;

/**
 * This function is effectively an interrupt, and is called
 * every time data is received from the I2C comms
 * @param byteCount how many bytes were sent, we don't use this value.
 */
void fromI2C(const size_t byteCount) {
    static char buffer[64];
    static char match_buffer[20];
    static size_t i = 0;
    static boolean seenAStartBit = false;
    boolean parseNumber = false;

    // Parse all data from the Wire, until we hit a stop point
    while (Wire.available()) {
        char c = Wire.read();
        Serial.print(c);

        // 'F' means 'Flush' the buffer
        if (c == 'F') {
            clear_point_queue();
            continue;
        }

        // 'A' and 'D' are the 'start bits'
        // these also determine what's been sent: angle, or distance info
        if ((c != 'A' && c != 'D' && c != 'P') && !seenAStartBit)
            continue;

        seenAStartBit = true;

        // 'S' is the 'stop' bit
        if (c == 'S') {
            seenAStartBit = false;
            parseNumber = true;
            break;
        }


        buffer[i] = c;
        i++;
        i %= 64;
    }

    // Once we received an 'S', either an angle or distance has been sent
    // these are of the form: A<angle>S, where <angle> is a floating-point value representing radians
    // or: D<distance>S, where <distance> is a floating-point value representing meters
    if (parseNumber) {
        Serial.println();

        // To parse the float, we use a clever trick: strtod() converts a C-style string
        // to a double-precision float. This means we need to:
        // (1) Use our char[] buffer as a C-style string (i.e. terminate w/ a null).
        // (2) Ensure only the <angle> or <distance> is in the buffer, not the 'A', 'D', or 'S'.
        buffer[i] = '\0'; // null-terminate the string
        i++;
        double parsedDouble = strtod(buffer+1, nullptr); // Skip over the 'A' or 'D'

        // Based on the value in buffer[0], the parsed number is either an angle
        // or a distance. Set the values appropriately
        switch (buffer[0]) {
            case 'A':
                receivedAngleFromPi = true;
                angleFromPi = parsedDouble;
                break;

            case 'D':
                receivedDistanceFromPi = true;
                distanceFromMarker = parsedDouble;
                break;

            // Parse the points
            case 'P':
                // DFA for the regular expression: \((\d+.\d+),(\d+.\d+)\)
                typedef enum {MATCH_LEFT, MATCH_UNTIL_COMMA, MATCH_UNTIL_RIGHT} regex_fsm_t;
                regex_fsm_t regex_state = MATCH_LEFT;
                bool matchError = false;
                size_t matchIndex = 1; // Skip the 'P'
                size_t matchBufferIndex = 0;
                double parsed_rho = 0.0;
                double parsed_phi = 0.0;

                // Match until we run out of matches
                while (buffer[matchIndex] != '0' && !matchError) {
                    switch (regex_state) {
                        // Match '(' then move on else error
                        case MATCH_LEFT: {
                            if (buffer[matchIndex] == '(') {
                                regex_state = MATCH_UNTIL_COMMA;
                                matchBufferIndex = 0;
                                matchIndex ++;
                            } else {
                                matchError = true;
                            }

                            break;
                        }

                        // Track the indices until we get to a ',', then match
                        case MATCH_UNTIL_COMMA: {
                            //  Once we see a comma, we've finished the rho match
                            if (buffer[matchIndex] == ',') {
                                // Add the '\0' to terminate this string, then strtod()
                                match_buffer[matchBufferIndex] = '\0';
                                parsed_rho = strtod(match_buffer, nullptr);

                                regex_state = MATCH_UNTIL_RIGHT;
                                matchIndex ++;
                                matchBufferIndex = 0;
                                break;
                            }

                            match_buffer[matchBufferIndex] = buffer[matchIndex];
                            matchBufferIndex++;
                            matchIndex ++;
                            break;
                        }

                        // Track until we hit a ')', then match
                        case MATCH_UNTIL_RIGHT: {
                            if (buffer[matchIndex] == ')') {
                                // Add the '\0' to terminate this string, then strtod()
                                match_buffer[matchBufferIndex] = '\0';
                                parsed_phi = strtod(match_buffer, nullptr);

                                // Add the point
                                add_point_to_buffer(parsed_rho, parsed_phi);

                                regex_state = MATCH_LEFT;
                                matchBufferIndex = 0;
                                matchIndex ++;
                                break;
                            }

                            match_buffer[matchBufferIndex] = buffer[matchIndex];
                            matchBufferIndex++;
                            matchIndex ++;
                            break;
                        }

                        default: {
                            Serial.println("regex_fsm_t hit default???\r\n");
                            break;
                        }
                    }
                }


        }

        i = 0;

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

/**
 * The main controller for a phi change. Uses both Kp and Ki
 * @param  currentAngle the current angle of the robot
 * @param  setAngle     the set angle of the robot
 * @return              the voltage value
 */
double turnController(const double currentAngle, const double setAngle) {
    const double Kp = 15;
    const double Ki = 150;
    const double fromPController = pController(currentAngle, setAngle, Kp);
    const double fromIController = iController(currentAngle, setAngle, turnControllerSum, Ki);

    const double setVoltage = fromPController + fromIController;

    return boundValue(setVoltage, NOMINAL_VOLTAGE, -NOMINAL_VOLTAGE);
}

/**
 * The main controller for forward movement
 * @param  currentRadius the current forward movement
 * @param  setRadius     the set forward movement
 * @return               the voltage value
 */
double radiusController(const double currentRadius, const double setRadius) {
    const double Kp = 15;

    const double Ki = 220;
    static double sum = 0;

    const double fromPController = pController(currentRadius, setRadius, Kp);
    const double fromIController = iController(currentRadius, setRadius, sum, Ki);
    const double setVoltage = fromPController + fromIController;

    return (setVoltage > NOMINAL_VOLTAGE ? NOMINAL_VOLTAGE : setVoltage);
}

// The main loop() has a state machine, so this is the TYPEDEF for that machine.
typedef enum { SPIN_TO_ARUCO, MOVE_TO_ARUCO, UPDATE_SET_POINT, SPIN_TO_SET_ANGLE, MOVE_TO_SET_RADIUS, SEND_H_TO_PI, IDLE } fsm_t;
void loop() {
    const unsigned long start_time = millis();
    static fsm_t state = UPDATE_SET_POINT; // Default state: spin until you see the aruco marker

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
        Serial.print(state);
        Serial.print("\t");
    #endif

    // Then state machine this
    switch (state) {
        case SPIN_TO_ARUCO: {
            typedef enum { NOMINAL_SPIN, WAIT_FOR_ANGLE_TO_STABALIZE, CORRECT_ANGLE } spin_to_aruco_fsm_t;
            static spin_to_aruco_fsm_t spin_aruco_state = NOMINAL_SPIN;
            static double setAngle = 0;

            switch (spin_aruco_state) {
                // Spin at a nominal voltage until we get any reading from the Aruco marker
                case NOMINAL_SPIN: {
                    if (receivedAngleFromPi) { // Halt once we've received a single angle and wait to stabalize
                        setMotorsVoltage(0, 0);
                        spin_aruco_state = WAIT_FOR_ANGLE_TO_STABALIZE;
                        break;
                    }

                    const double deltaVa = 4;
                    const double Va = 0;
                    setMotorsVoltage(Va, deltaVa);
                    break;
                }

                // Once we've received a single reading, halt and wait for the measurement to stabalize
                case WAIT_FOR_ANGLE_TO_STABALIZE: {
                    static double previousMeasurement = 10.0;

                    // If the current measurement is "close enough" to the previous measurement,
                    // then go to correct angle
                    if (receivedAngleFromPi) {
                        if (withinEpsilon(previousMeasurement, angleFromPi, 0.05)) {
                            // Update the set angle w/ a small correction factor
                            // either add +0.1 if the pi has overshot and needs to correct
                            // or subtract 0.05 if the pi has undershot
                            // These values were found via tuning
                            setAngle = angleFromPi + (angleFromPi < 0 ? 0.1 : -0.05);
                            spin_aruco_state = CORRECT_ANGLE;
                            break;
                        } else {
                            // New angle isn't close enough, keep waiting
                            receivedAngleFromPi = false;
                            previousMeasurement = angleFromPi;
                        }
                    }

                    // Since we're not moving, keep the encoder counts at 0
                    rightWheel1.write(0);
                    leftWheel2.write(0);

                    setMotorsVoltage(0, 0);
                    break;
                }

                // Once the angle has stabalized, go to the correct angle
                case CORRECT_ANGLE: {
                    // If we get close enough, call it good and go to MOVE_TO_ARUCO.
                    if (withinEpsilon(setAngle, currentAngle, 0.01)) {
                        setMotorsVoltage(0, 0);
                        state = MOVE_TO_ARUCO;
                        break;
                    }

                    const double deltaVa = turnController(currentAngle, setAngle);
                    const double Va = 0;
                    setMotorsVoltage(deltaVa, Va);
                    break;
                }

                default: {
                    Serial.println("SPIN_TO_ARUCO FSM hit default??\r\n");
                    break;
                }
            }

            break;
        }

        // Moves to within 1 foot of the aruco marker
        // ...which is really just enquing the point { distance, 0 } to the queue
        case MOVE_TO_ARUCO: {
            const double distanceToMove = distanceFromMarker - 0.1;
            Serial.print("Move to: ");
            Serial.println(distanceToMove);

            // Enqueue the distance
            pointQueue.enqueue({ distanceToMove, 0 });

            // Enqueue all the points around the box.
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
                state = SEND_H_TO_PI;
                break;
            }

            const point new_point = get_next_point();
            piSetRadius = new_point.radius;
            piSetAngle = new_point.angle;

            state = SPIN_TO_SET_ANGLE;
            rightWheel1.write(0);
            leftWheel2.write(0);
            turnControllerSum = 0;
            break;
        }

        /**
         * Only spin towards the robot, with no lateral movement
         * The angle we should aim towards is based on currentX/Y and the setX/Y
         */
        case SPIN_TO_SET_ANGLE: {
            const double setAngle = piSetAngle;

            // If we are "close enough" to the actual angle we need to face, then go to MOVE state
            if (withinEpsilon(setAngle, currentAngle, 0.01)) {
                state = MOVE_TO_SET_RADIUS;
                setMotorsVoltage(0, 0); // Turn off motors on transition to prevent over-turn.
                delay(200);
                piSetAngle = 0.0;
                rightWheel1.write(0);
                leftWheel2.write(0);
                turnControllerSum = 0;
                break;
            }

            // Else, turn until we get there.
            const double deltaVa = turnController(currentAngle, setAngle);
            const double Va = 0;

            setMotorsVoltage(Va, deltaVa);
            break;
        }

        /**
         * State MOVE_TO_SET_RADIUS: move until we've moved the correct radius
         */
        case MOVE_TO_SET_RADIUS: {
            // Hold the angle steady, while we move steadily forward _to_ the set radiuan
            const double setAngle = piSetAngle;

            // If we get close enough, head to the next point.
            // This is technically a bang-bang controller. But I think it works.
            if (withinEpsilon(currentRadius, piSetRadius, 0.01)) {
                setMotorsVoltage(0, 0);
                delay(100);
                state = UPDATE_SET_POINT;
                rightWheel1.write(0);
                leftWheel2.write(0);
                break;
            }

            // We use a turn controller to ensure we don't drift from the setAngle
            // While we move at a nominal rate to the set point.
            const double deltaVa = turnController(currentAngle, setAngle); // pController(currentAngle, setAngle, Kp);
            const double Va = radiusController(currentRadius, piSetRadius); //;NOMINAL_VOLTAGE;
            setMotorsVoltage(Va, deltaVa);
            break;
        }

        /**
         * When we've ran out of points, send the 'H' and then go to IDLE.
         */
        case SEND_H_TO_PI: {
            // TODO: Send the H
            state = IDLE;
            setMotorsVoltage(0, 0);
            break;
        }

        /**
         * Wait in IDLE until we receive another point
         */
        case IDLE: {
            setMotorsVoltage(0, 0);

            // If we have received another point, go back to UPDATE_SET_POINT
            if (has_another_point()) {
                state = UPDATE_SET_POINT;
                break;
            }

            state = IDLE;
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
