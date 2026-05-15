/*
 * sensors.cpp — Implementation of the 5-channel IR tracker reader.
 *
 * The tracker boards from OSOYOO are digital: each channel's OUT pin sits
 * at a steady HIGH or LOW depending on whether it currently sees the line.
 * Which level means "line detected" depends on the line/background colors
 * and the board revision, so the active level is configurable via the
 * IR_LINE_ACTIVE macro in config.h.
 */

#include "sensors.h"
#include "config.h"

// Pin list kept in channel order so loops can index it directly.
static const uint8_t IR_PINS[5] = {
    IR_CH0_PIN,
    IR_CH1_PIN,
    IR_CH2_PIN,
    IR_CH3_PIN,
    IR_CH4_PIN,
};

void sensorsInit() {
    for (uint8_t i = 0; i < 5; i++) {
        pinMode(IR_PINS[i], INPUT);
    }
}

IrReading readIr() {
    IrReading r;
    r.anyDetected = false;

    // Sum of (channel index) for every lit channel, plus a count, so we
    // can take the average to find where the line is centered laterally.
    int   indexSum = 0;
    uint8_t litCount = 0;

    for (uint8_t i = 0; i < 5; i++) {
        bool lit = (digitalRead(IR_PINS[i]) == IR_LINE_ACTIVE);
        r.ch[i] = lit;
        if (lit) {
            indexSum += i;
            litCount++;
            r.anyDetected = true;
        }
    }

    if (litCount == 0) {
        // No channel sees the line — caller should check anyDetected
        // before steering on this value.
        r.weightedPosition = 0;
    } else {
        // Average lit index is in 0..4; shift to -2..+2 so 0 means centered.
        // Integer division is fine here because we only need integer-grade
        // resolution for steering decisions; if you want finer control,
        // switch to float math and return a float instead.
        int average = indexSum / litCount;
        r.weightedPosition = (int8_t)(average - 2);
    }

    return r;
}
