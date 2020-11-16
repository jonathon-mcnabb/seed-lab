/**
 * Handles all the stuff with the (rho, phi) points buffer.
 * - Currently, points are manually added since we know the path (i.e. the course doesn't change)
 * - For the final demo, these points will be loaded from I2C
 */

#pragma once

#include <ArduinoQueue.h>

struct point {
    double radius;
    double angle;
};

// The queue itself: since Arduino doesn't like dynamic allocation, we have to set a max size
// https://github.com/EinarArnason/ArduinoQueue is the documentation for this library
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

/**
 * Called on 'F', clears the point buffer
 */
void clear_point_queue() {
    while (!pointQueue.isEmpty()) {
        pointQueue.dequeue();
    }
}

/**
 * Adds a point to the queue
 * @param rho rho / radius, meters
 * @param phi phi / angle, radians
 */
void add_point_to_buffer(const double rho, const double phi) {
    pointQueue.enqueue({ rho, phi });

    Serial.print("Added: ");
    Serial.print(rho);
    Serial.print(", ");
    Serial.print(phi);
    Serial.println(" to the buffer");
}
