#include "functions.h"
#include "constants.h"

/**
 * Runs the inner controller: given the desired PhiDot (turning speed), and the current, run the Ki term
 * @param phiDotActual the current PhiDot
 * @param phiDotSet    the desired PhiDot
 */
double innerPhiDotController(double phiDotActual, double phiDotSet) {
    static const double Ki = 900; //900;
    static const double Kp = 5.5; //5.5;
    static double sum = 0;

    // Serial.println("X");

    // return pController(phiDotActual, phiDotSet, sum, Ki);
    double output = pController(phiDotActual, phiDotSet, Kp);
    output += iController(phiDotActual, phiDotSet, sum, Ki);
    return boundValue(output, BATTERY_MAX_VOLTAGE, -1*BATTERY_MAX_VOLTAGE);
}

/**
 * Runs the outer loop of the phi controller
 * @param  phiActual  current angle
 * @param  phiDesired set angle
 * @return
 */
double outerPhiController(double phiActual, double phiDesired) {
    static const double Kp = 10;

    double result = pController(phiActual, phiDesired, Kp);

    // Limit this output... to what?
    static const double limit = 4; // seems to be about the max.
    return boundValue(result, limit, -1*limit);
}

/**
 * The whole shebang
 * @param  phiDotActual the current phi-velocity
 * @param  phiActual    the current angle, phi
 * @return
 */
double completePhiController(double phiDotActual, double phiActual) {
    // get the result from the outer controller
    double phiDotDesired = outerPhiController(phiActual, SET_PHI);

    // Serial.print(phiActual);
    // Serial.print("\t");
    // Serial.print(phiDotDesired);
    // Serial.print("\t");
    // Serial.print(phiDotActual);
    // Serial.println();

    // return the result from the inner controller
    return innerPhiDotController(phiDotActual, phiDotDesired);
}
