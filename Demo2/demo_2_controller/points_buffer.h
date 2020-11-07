/**
 * Handles all the stuff with the (X, Y) points buffer
 */

#pragma once

#include <ArduinoQueue.h>

struct point {
    double x;
    double y;
};

// The queue itself: since Arduino doesn't like dynamic allocation, we have to set a max size
// https://github.com/EinarArnason/ArduinoQueue
ArduinoQueue<point> pointQueue(50);

/**
 * Returns whether the queue has another point to give
 * @return true/false if yes/no
 */
bool has_another_point() {
    return !pointQueue.isEmpty();
}

/**
 * Returns the top point, and remove from queue
 * @return the next point
 */
point get_next_point() {
    return pointQueue.dequeue();
}

// The main shebang: hook serialEvent(), and automatically add to the queue
// we expect either a list of tuples, e.g.: (X.XX,Y.YY)(X.XX,Y.YY)
// or FLUSH, followed by at least 1 tuple, e.g.: F(X.XX,Y.YY)
// so no matter what, only use this "interrupt" when we have at least 11 points
// and the first character matches one of our cases
void serialEvent() {
    // Do nothing if it doesn't match our protocol
    if (Serial.available() < 11 || (Serial.peek() != 'F' && Serial.peek() != '(')) {
        Serial.println("Data in serialEvent doesn't match expected, ignoring.");
        return;
    }

    // If we get a FLUSH, clear the pointQueue
    if (Serial.peek() == 'F') {
        while (!pointQueue.isEmpty()) {
            pointQueue.dequeue();
        }
    }

    // Then, parse each tuple, and append
    while (Serial.available() >= 11) {
        // Have some error detection
        if (Serial.peek() != '(') {
            Serial.print("Unexpected token: ");
            Serial.println(Serial.peek());
            break;
        }

        double x = Serial.parseFloat();

        if (Serial.peek() != ',') {
            Serial.print("Unexpected token: ");
            Serial.println(Serial.peek());
            break;
        }

        double y = Serial.parseFloat();

        if (Serial.peek() != ')') {
            Serial.print("Unexpected token: ");
            Serial.println(Serial.peek());
            break;
        }

        Serial.read();
        // TODO: construct point & push
        pointQueue.enqueue({ x, y});
    }
}
