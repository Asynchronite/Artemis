/*
 * display.h — LCD output helpers for the Artemis robot.
 *
 * Wraps the LiquidCrystal_I2C library so the rest of the code never has to
 * deal with the LCD object directly. The headline feature is lcdPrintLine():
 * it overwrites a full row in one shot, padding with spaces to the LCD's
 * width so leftover characters from a previous (longer) message disappear
 * automatically. That means no flickering lcd.clear() before every update.
 *
 * Typical usage:
 *     displayInit();
 *     lcdPrintLine(0, "Hello");        // writes "Hello           "
 *     lcdPrintLine(0, "Hi");           // writes "Hi              " — no leftovers
 *     lcdPrintAt(12, 1, "OK");         // writes "OK" at column 12 of row 1 (no padding)
 */

#ifndef ARTEMIS_DISPLAY_H
#define ARTEMIS_DISPLAY_H

#include <Arduino.h>

// Initialize the I2C LCD: bring up the bus, turn on the backlight, and
// leave the screen blank. Call once from setup() before any other lcd*
// function.
void displayInit();

// Overwrite an entire LCD row with `text`. Padding/truncation handled
// internally so the row always ends up exactly LCD_COLS characters wide,
// erasing anything that was there before. No need to clear first.
//
//   row: 0 = top row, 1 = bottom row (on a 16x2 panel).
void lcdPrintLine(uint8_t row, const String& text);

// Write `text` starting at (col, row) without padding or clearing. Useful
// for updating a small region (e.g. a single value in a label/value pair)
// while leaving the rest of the line untouched.
void lcdPrintAt(uint8_t col, uint8_t row, const String& text);

// Wipe every cell on the screen. Most code should prefer lcdPrintLine()
// instead, since this causes a visible flicker on cheap I2C panels.
void lcdClear();

#endif // ARTEMIS_DISPLAY_H
