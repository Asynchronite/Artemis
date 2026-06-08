/*
 * ultrasonic.cpp — NewPing-backed HC-SR04 readers (front, left, right).
 *
 * One NewPing instance per sensor, held in an array indexed by the
 * UltrasonicSensor enum (front, left, right — there is no rear). Each instance
 * configures its own trigger/echo pins from its constructor, so there is
 * nothing to do in setup(). readDistance() takes a single ping on the named
 * sensor and reports the raw echo time alongside the converted distances.
 * NewPing returns an echo time of 0 when nothing answers within
 * ULTRASONIC_MAX_CM, which we surface as inRange == false.
 */

#include "ultrasonic.h"
#include "config.h"
#include <NewPing.h>

// One sensor per UltrasonicSensor value, in the same order as the enum so the
// enum can index this array directly.
static NewPing sonar[US_COUNT] = {
    NewPing(ULTRASONIC_FRONT_TRIG_PIN, ULTRASONIC_FRONT_ECHO_PIN, ULTRASONIC_MAX_CM),
    NewPing(ULTRASONIC_LEFT_TRIG_PIN,  ULTRASONIC_LEFT_ECHO_PIN,  ULTRASONIC_MAX_CM),
    NewPing(ULTRASONIC_RIGHT_TRIG_PIN, ULTRASONIC_RIGHT_ECHO_PIN, ULTRASONIC_MAX_CM),
};

UltrasonicReading readDistance(UltrasonicSensor sensor) {
    UltrasonicReading r;

    // ping_median() fires ULTRASONIC_PING_SAMPLES pulses (~29 ms apart) and
    // returns the median round-trip time in microseconds, or 0 if no echo came
    // back before the MAX_CM cutoff. The median rejects one-off spikes and the
    // cross-talk between sensors that otherwise shows up as a phantom near wall.
    r.echoUs = sonar[sensor].ping_median(ULTRASONIC_PING_SAMPLES);

    // Convert the echo time into distances. Both return 0 when the echo timed
    // out, matching echoUs == 0.
    r.cm     = sonar[sensor].convert_cm(r.echoUs);
    r.inches = sonar[sensor].convert_in(r.echoUs);

    r.inRange = (r.echoUs != 0);

    return r;
}
