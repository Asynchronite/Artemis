/*
 * sensors.h — OSOYOO 5-channel IR tracker reader.
 *
 * The tracker has five infrared sensors arranged in a horizontal strip. Each
 * channel outputs a digital signal that flips state when it passes over the
 * line. readIr() reads all five channels at once and bundles the result
 * (plus a useful "where is the line, roughly?" summary) into an IrReading.
 *
 * Channel-to-position mapping (looking down at the robot from above with
 * the front pointing away from you):
 *
 *     ch[0]  ch[1]  ch[2]  ch[3]  ch[4]
 *      -2     -1      0     +1     +2     <- weight contributing to position
 *      far    left   center right  far
 *      left                              right
 *
 * weightedPosition is the average index of the channels that see the line,
 * remapped to the range -2..+2:
 *   -2 = line is fully to the left      (only ch[0] lit)
 *    0 = line is dead center            (only ch[2] lit, or symmetric mix)
 *   +2 = line is fully to the right     (only ch[4] lit)
 * If no channel sees the line, anyDetected is false and weightedPosition is
 * 0 — use anyDetected to distinguish "centered" from "lost". This is the
 * value you typically feed into a proportional steering controller.
 */

#ifndef ARTEMIS_SENSORS_H
#define ARTEMIS_SENSORS_H

#include <Arduino.h>

struct IrReading {
    bool   ch[5];             // true = this channel currently sees the line
    int8_t weightedPosition;  // -2..+2, where 0 means line is centered
    bool   anyDetected;       // false if every channel reports "no line"
};

// Configure the five IR sensor pins as inputs. Call once from setup().
void sensorsInit();

// Sample all five channels and return a fresh reading. Cheap (~5 digital
// reads), safe to call every iteration of loop().
IrReading readIr();

#endif // ARTEMIS_SENSORS_H
