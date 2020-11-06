/**
  * This file contains the complete rho_controller
  *
  * @author Luke Henke
  *
  * Class: Seed Lab.
  */

#include "functions.h"
#include "constants.h"

/**
 * Runs the inner controller: given the desired PhiDot (turning speed), and the current, run the Ki term
 * @param rhoDotActual the current PhiDot
 * @param rhoDotSet    the desired PhiDot
 */
double innerRhoDotController(double rhoDotActual, double rhoDotSet) {
    static const double Ki = 10;
    static const double Kp = 15;
    static double sum = 0;

    // return pController(rhoDotActual, rhoDotSet, sum, Ki);
    double output = pController(rhoDotActual, rhoDotSet, Kp);
    output += iController(rhoDotActual, rhoDotSet, sum, Ki);
    return boundValue(output, BATTERY_MAX_VOLTAGE, -1*BATTERY_MAX_VOLTAGE);
}

/**
 * Runs the outer loop of the rho controller
 * @param  rhoActual  current meters forward
 * @param  rhoDesired set meters forward
 * @return
 */
double outerRhoController(double rhoActual, double rhoDesired) {
    static const double Kp = 15;

    double result = pController(rhoActual, rhoDesired, Kp);

    // Limit this output... to what?
    static const double limit = 4; // seems to be about the max.
    return boundValue(result, limit, -1*limit);
}

/**
 * The whole shebang
 * @param  rhoDotActual the current rho-velocity
 * @param  rhoActual    the current angle, rho
 * @return
 */
double completeRhoController(double rhoDotActual, double rhoActual) {
    // get the result from the outer controller
    double rhoDotDesired = outerRhoController(rhoActual, SET_RHO);

    // return the result from the inner controller
    double output = innerRhoDotController(rhoDotActual, rhoDotDesired);
    return output;
}
