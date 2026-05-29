/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____
 * / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \
 *| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
 * \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
 *                  (____/
 *
 * Artemis — line-following maze robot.
 *   Board:   Arduino Mega 2560
 *   Drive:   4 mecanum wheels via two MODEL-X H-bridge boards
 *   Sensor:  OSOYOO 5-channel IR line tracker
 *   Display: I2C 16x2 LCD (PCF8574 backpack)
 *
 * Each subsystem lives in its own module:
 *   config.h   — every pin number and tunable constant
 *   motors.*   — drive control (go_advance, left_turn, ...)
 *   sensors.*  — IR tracker reading (readIr() returns an IrReading)
 *   display.*  — non-flickering LCD helpers (lcdPrintLine, lcdPrintAt)
 *
 * This file is intentionally short: it just wires everything together and
 * leaves the maze-solving brain inside loop() as a TODO for you to fill in.
 */

#include <Arduino.h>
#include "config.h"
#include "motors.h"
#include "display.h"
#include "sensors.h"

void setup() {
    Serial.begin(9600);

    motorsInit();
    displayInit();
    sensorsInit();

    lcdPrintLine(0, "Artemis ready");
    lcdPrintLine(1, "v0.1");
}

void loop() {
    // 1. Read the line tracker. ir.ch[0..4] are the per-channel hits,
    //    ir.weightedPosition is -2..+2 (negative = line is to the left),
    //    ir.anyDetected is false when no channel sees the line at all.
    IrReading ir = readIr();

    // 2. Show the live sensor state on the LCD so you can debug without a
    //    serial cable. Both lines are overwritten in place — no flicker.
    String pattern = "IR ";
    for (uint8_t i = 0; i < 5; i++) pattern += ir.ch[i] ? 'X' : '.';
    lcdPrintLine(0, pattern);
    lcdPrintLine(1, ir.anyDetected
                        ? String("pos=") + ir.weightedPosition
                        : String("pos=?  (no line)"));

    // 3. TODO: maze-solving / line-following logic goes here.
    //    Suggested building blocks:
    //      - Steer with ir.weightedPosition (e.g. proportional control:
    //          if (ir.weightedPosition < 0) right_turn(slowSpeed);
    //          else if (ir.weightedPosition > 0) left_turn(slowSpeed);
    //          else go_advance(cruiseSpeed);
    //        — but FIRST verify in your hardware which side maps to which
    //        sign; swap the comparisons if your sensor is mounted flipped.)
    //      - Detect intersections by looking at how many channels are lit
    //        at once (e.g. 4-5 lit channels usually means a perpendicular
    //        cross-line, so you've reached a junction).
    //      - React to losing the line (ir.anyDetected == false): stop,
    //        back up, or rotate to search.
    //    Helpers available from motors.h:
    //      go_advance(speed), go_back(speed),
    //      left_turn(speed), right_turn(speed),
    //      left_shift(...), right_shift(...),
    //      clockwise(speed), countclockwise(speed),
    //      stop_Stop().

    // 4. Small idle delay to keep the LCD refresh rate sane and to give
    //    the IR sensor a moment between reads. Drop this once you start
    //    driving the motors and need tighter timing.
    delay(50);
}
