/*
 * ultrasonic.h — HC-SR04 distance sensing for the Artemis robot.
 *
 * Wraps the NewPing library for the robot's three HC-SR04 ultrasonic
 * rangefinders — front, left, and right. (There is deliberately no rear
 * sensor.) Each call to readDistance() fires one named sensor and bundles the
 * result.
 *
 * A reading tells you how far away the nearest object is in that sensor's
 * direction. Anything nearer than the sensor's minimum or farther than
 * ULTRASONIC_MAX_CM (see config.h) comes back as "out of range" — always check
 * inRange before trusting the distance, since 0 is also what you get when
 * nothing answers.
 *
 * NewPing configures the trigger/echo pins from its constructors, so there is
 * no init() to call from setup(): just start calling readDistance().
 */

#ifndef ARTEMIS_ULTRASONIC_H
#define ARTEMIS_ULTRASONIC_H

#include <Arduino.h>

// The robot's three rangefinders. US_COUNT is the number of sensors and also
// doubles as the size of the internal NewPing array — keep it last.
enum UltrasonicSensor {
    US_FRONT = 0,
    US_LEFT,
    US_RIGHT,
    US_COUNT
};

struct UltrasonicReading {
    unsigned int echoUs;   // raw round-trip echo time in microseconds (0 = no echo)
    unsigned int cm;       // distance to the nearest object, centimetres
    unsigned int inches;   // distance to the nearest object, inches
    bool         inRange;  // false when nothing was detected within range
};

// Fire one sensor and return its distance. Blocks only for as long as it takes
// sound to travel out to ULTRASONIC_MAX_CM and back (a few ms at most). NewPing
// enforces a short minimum gap between pings on a given sensor internally, so
// reading each sensor once per loop() iteration is safe.
UltrasonicReading readDistance(UltrasonicSensor sensor);

#endif // ARTEMIS_ULTRASONIC_H
