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
#define SPEED       50
#define TURN_SPEED  50

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
//  Ultrasonic distance sensors (3x HC-SR04, driven by the NewPing library)
// =====================================================================
// Three HC-SR04s — front, left, and right. There is deliberately no rear
// sensor. Each one needs its own trigger and echo line; NewPing pulses TRIG
// and times the echo for us. All six pins below are free of the motor and I2C
// assignments above. Front keeps the 48/49 pair from the standalone
// ultrasound bring-up sketch.
#define ULTRASONIC_FRONT_TRIG_PIN  47
#define ULTRASONIC_FRONT_ECHO_PIN  46
#define ULTRASONIC_LEFT_TRIG_PIN   49
#define ULTRASONIC_LEFT_ECHO_PIN   48
#define ULTRASONIC_RIGHT_TRIG_PIN  51
#define ULTRASONIC_RIGHT_ECHO_PIN  50

// Maximum range to listen for, in centimetres, shared by all three sensors.
// NewPing stops waiting for an echo past this distance (reporting the reading
// as out-of-range), which also caps how long a single ping can block. 400 cm
// is the HC-SR04's practical ceiling.
#define ULTRASONIC_MAX_CM    400

// How many pulses to take per reading. NewPing's ping_median() fires this many
// pings ~29 ms apart and returns the MIDDLE value, which throws out one-off
// spikes and — crucially — the cross-talk you get when two nearby walls echo
// into a third sensor (the usual cause of a phantom "wall" on an open side, and
// of a surprise SPIN). 1 = single fast ping; 3 is a robust default; higher is
// steadier but slower (each extra sample adds ~29 ms to that sensor's read).
#define ULTRASONIC_PING_SAMPLES  3

// =====================================================================
//  Maze-following behaviour (consumed by loop() in src/main.cpp)
// =====================================================================
// Strategy selector — change THIS ONE LINE and re-upload to switch how the
// robot solves the maze:
//     MAZE_GREEDY      steer toward whichever direction is most open (default)
//     MAZE_RIGHT_HAND  keep the right-hand wall (classic maze solver)
//     MAZE_LEFT_HAND   keep the left-hand wall (mirror image)
#define MAZE_GREEDY      0
#define MAZE_RIGHT_HAND  1
#define MAZE_LEFT_HAND   2
#define MAZE_STRATEGY    MAZE_GREEDY

// SAFETY SWITCH. Set to 0 to run the full sense/decide/display loop with the
// motors held STOPPED — perfect for bench-testing sensors, the LCD and the
// decision logic without the robot driving off the table. Set to 1 once it's
// in the maze and you actually want it to move.
#define MAZE_DRIVE_MOTORS    1   // 0 = bench-safe (no motion); set to 1 in the maze

// Forward / turning speeds while solving (PWM duty cycle, 0–255). A touch
// brisker than the SPEED demo default for responsiveness; lower them if the
// robot overshoots corners.
#define MAZE_CRUISE_SPEED   100
#define MAZE_TURN_SPEED     120

// A FRONT reading at or below this many centimetres counts as "wall ahead —
// stop driving straight and turn." Tune to the robot's stopping distance.
#define FRONT_STANDOFF_CM    25

// A side needs at least this much room (cm) to count as an opening worth
// turning into. Make it comfortably wider than the robot's body.
#define SIDE_CLEARANCE_CM    22

// Greedy hysteresis: only veer toward a side instead of driving straight when
// that side has at least this many EXTRA centimetres of room over the front.
// Stops the robot wiggling when front and side clearances are nearly equal.
#define TURN_BIAS_CM         25

// Corner turns are a simple, predictable STOP -> PAUSE -> TURN -> PAUSE -> go
// sequence, so the motion is easy to watch and calibrate:
//   1. the moment a corner is detected (wall ahead + an open side), the robot
//      STOPS dead,
//   2. waits TURN_PAUSE_MS so momentum settles,
//   3. spins in place for exactly TURN_TIME_MS (a fixed time, not sensor-based),
//   4. STOPS and waits TURN_PAUSE_MS again,
//   5. drives FORWARD for POST_TURN_FWD_MS to pull clear of the junction, then
//   6. resumes following.
// It never drives forward while a wall is straight ahead, so it can't knock a
// wall down: if one timed turn isn't quite enough, it just stops and does
// another the same way.
//
// Step 5 fixes the "turns into the same opening twice" problem: right after a
// turn the side sensor still sees the corridor it just came out of, so a plain
// follower would immediately turn back into it. Driving forward first carries
// the (rear-mounted) side sensors past the junction before it looks again.
//
//   TURN_TIME_MS     : how long the spin lasts. Calibrate to about a 90° turn at
//                      MAZE_TURN_SPEED — RAISE if it under-rotates (turns too
//                      little), LOWER if it over-rotates.
//   TURN_PAUSE_MS    : the short settle time it holds still before and after the
//                      turn. Keep it small.
//   POST_TURN_FWD_MS : how long to drive straight after a turn before it may turn
//                      again. RAISE if it still turns back into the junction;
//                      LOWER if it overruns a real turn just past a corner. If a
//                      wall comes up ahead first, it stops early and re-decides.
#define TURN_TIME_MS      750
#define TURN_PAUSE_MS     1000
#define POST_TURN_FWD_MS  1000

#endif // ARTEMIS_CONFIG_H
