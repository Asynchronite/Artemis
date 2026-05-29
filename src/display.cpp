/*
 * display.cpp — LCD wrapper implementation.
 *
 * The LiquidCrystal_I2C object is kept file-static so no other module can
 * accidentally talk to the LCD directly — everything has to go through the
 * helpers declared in display.h. That keeps formatting consistent (every
 * write goes through the same padding logic) and makes it easy to swap the
 * underlying library later if needed.
 */

#include "display.h"
#include "config.h"
#include <LiquidCrystal_I2C.h>

// Single LCD instance for the whole program. Configured from constants in
// config.h so the wiring is documented in exactly one place.
static LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

void displayInit() {
    lcd.init();
    lcd.backlight();
    lcd.clear();
}

void lcdPrintLine(uint8_t row, const String& text) {
    // Refuse to write to a row that doesn't exist on this panel — silently
    // ignoring is safer than letting the LCD library wrap to row 0 with
    // garbage column coordinates.
    if (row >= LCD_ROWS) return;

    // Build a fixed-width line: either truncate if the caller passed in
    // something too long, or pad with spaces so any previous characters in
    // this row get overwritten.
    char buffer[LCD_COLS + 1];
    size_t textLen = text.length();
    for (uint8_t i = 0; i < LCD_COLS; i++) {
        buffer[i] = (i < textLen) ? text[i] : ' ';
    }
    buffer[LCD_COLS] = '\0';

    lcd.setCursor(0, row);
    lcd.print(buffer);
}

void lcdPrintAt(uint8_t col, uint8_t row, const String& text) {
    if (row >= LCD_ROWS || col >= LCD_COLS) return;
    lcd.setCursor(col, row);
    lcd.print(text);
}

void lcdClear() {
    lcd.clear();
}
