/**
 * To clean up the main file a little bit, this file exists to pull out generic functions.
 *
 * @author: Luke Henke
 *
 * Class: SEED lab
 */

// This file is included in multiple areas, so prevent redeclaration errors
#pragma once
#include "constants.h"

/**
 * Calculate V1 and V2 from Va and deltaVa
 * @param  Va      Pseudovelocity
 * @param  deltaVa Pseudovelocity
 * @return         V1 or V2
 */
double calculateV1(double Va, double deltaVa) {
    return (Va + deltaVa) / 2.0;
}

double calculateV2(double Va, double deltaVa) {
    return (Va - deltaVa) / 2.0;
}

/**
 * These two functions allow for the controller to be turned off
 * once the robot gets "close enough", i.e., it's "within epsilon"
 * of the set point
 * @param  currVal Current value
 * @param  setVal  Set Value
 * @return         true or false
 */
bool withinEpsilon(double currVal, double setVal) {
    return fabs(currVal - setVal) <= EPSILON;
}

bool withinEpsilon(double currVal, double setVal, double epsilon) {
    return fabs(currVal - setVal) <= epsilon;
}

/**
 * Bound constraints -- can't go above MAX or below MIN
 */
double boundValue(double value, double max, double min) {
    if (value >= max) return max;
    if (value <= min) return min;

    return value;
}

/**
 * Calculates the p-term of a controller
 * @param  currVal current value
 * @param  setVal set value
 * @param  Kp  Kp term
 * @return     result
 */
double pController(double newVal, double setVal, double Kp) {
    return (setVal - newVal) * Kp;
}

/**
 * Calculates the i-term of a controller
 * @param  newVal new value
 * @param  setVal set value
 * @param  sum    the current error sum -- THIS GETS UPDATED
 * @param  Ki     the Ki constant
 * @return        the result of the i-term
 */
double iController(double newVal, double setVal, double& sum, double Ki) {
    if (fabs(newVal - setVal) > 0.5) {
        sum = 0;
        return 0;
    }

    sum += (setVal - newVal); // update sum with the new error

    if (sum > SUM_MAX) {
        sum = SUM_MAX;
    } else if (sum < -1*SUM_MAX) {
        sum = -1*SUM_MAX;
    }
    return Ki * sum * PERIOD / 1e3; // ?? I think I need to convert to seconds?
}

/**
 * Converts [0, BATTERY_MAX_VOLTAGE] |-> [0, 255]
 * @param  voltage the input voltage
 * @return         the output PWM
 */
double voltageToPWM(double voltage) {
    voltage = fabs(voltage);

    // Ensure we don't go over 255
    if (voltage > BATTERY_MAX_VOLTAGE)
        voltage = BATTERY_MAX_VOLTAGE;

    return voltage * 255.0 / BATTERY_MAX_VOLTAGE;
}
