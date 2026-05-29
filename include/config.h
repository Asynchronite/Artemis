/*
 * config.h — Central hardware configuration for the Artemis robot.
 *
 * Every pin number, address, and tunable constant lives in this file so the
 * rest of the codebase never has to hard-code anything. Change wiring or
 * tweak speeds here and the modules pick it up automatically.
 */

#ifndef ARTEMIS_CONFIG_H
#define ARTEMIS_CONFIG_H

#include <Arduino.h>

// =====================================================================
//  Default motor speeds (PWM duty cycle, 0–255)
// =====================================================================
// Used by setup demos and as a sensible starting point for line-following.
#define SPEED       100
#define TURN_SPEED  100

// =====================================================================
//  Motor pins — front pair (driven by the RIGHT MODEL-X driver board)
// =====================================================================
#define speedPinR           9    // Front Right wheel PWM  -> Right MODEL-X ENA
#define RightMotorDirPin1   22   // Front Right direction 1 -> Right MODEL-X IN1 (K1)
#define RightMotorDirPin2   24   // Front Right direction 2 -> Right MODEL-X IN2 (K1)
#define LeftMotorDirPin1    26   // Front Left  direction 1 -> Right MODEL-X IN3 (K3)
#define LeftMotorDirPin2    28   // Front Left  direction 2 -> Right MODEL-X IN4 (K3)
#define speedPinL           10   // Front Left  wheel PWM   -> Right MODEL-X ENB

// =====================================================================
//  Motor pins — rear pair (driven by the LEFT MODEL-X driver board)
// =====================================================================
#define speedPinRB          11   // Rear Right wheel PWM   -> Left MODEL-X ENA
#define RightMotorDirPin1B  5    // Rear Right direction 1 -> Left MODEL-X IN1 (K1)
#define RightMotorDirPin2B  6    // Rear Right direction 2 -> Left MODEL-X IN2 (K1)
#define LeftMotorDirPin1B   7    // Rear Left  direction 1 -> Left MODEL-X IN3 (K3)
#define LeftMotorDirPin2B   8    // Rear Left  direction 2 -> Left MODEL-X IN4 (K3)
#define speedPinLB          12   // Rear Left  wheel PWM   -> Left MODEL-X ENB

// =====================================================================
//  LCD configuration (I2C 16x2 with PCF8574 backpack)
// =====================================================================
// On Arduino Mega the I2C bus uses SDA = pin 20 and SCL = pin 21.
// If the screen stays blank, try changing LCD_ADDR to 0x3F — that's the
// other common backpack address. (Run an I2C scanner if neither works.)
#define LCD_ADDR  0x27
#define LCD_COLS  16
#define LCD_ROWS  2

// =====================================================================
//  IR line-tracker pins (OSOYOO 5-channel digital tracker)
// =====================================================================
// Channels are numbered left-to-right when looking down at the robot from
// above with the front of the chassis pointing away from you:
//   CH0 = far left, CH2 = center, CH4 = far right.
// Wire each OUT pin from the tracker to one of these Mega digital inputs.
// TODO: replace the placeholder pin numbers below with your actual wiring.
#define IR_CH0_PIN  A0
#define IR_CH1_PIN  A1
#define IR_CH2_PIN  A2
#define IR_CH3_PIN  A3
#define IR_CH4_PIN  A4

// Most OSOYOO 5-channel trackers pull their OUT pin LOW when the channel is
// above a dark line on a light surface. If your line is light on dark, or
// the tracker's onboard inverter is wired the other way, change this to HIGH.
#define IR_LINE_ACTIVE  LOW

#endif // ARTEMIS_CONFIG_H
