/*
 * motors.h — Mecanum-wheel drive API for the Artemis robot.
 *
 * The robot has four independent motors (front/rear × left/right) driven by
 * two MODEL-X H-bridge boards. Each "speed" argument is a PWM duty cycle in
 * the range 0–255; 0 means stopped, 255 means full throttle.
 *
 * Naming convention:
 *   FR / FL / RR / RL = Front-Right / Front-Left / Rear-Right / Rear-Left
 *   _fwd / _bck       = forward / backward direction
 *
 * Call motorsInit() once from setup() before using any other function.
 */

#ifndef ARTEMIS_MOTORS_H
#define ARTEMIS_MOTORS_H

#include <Arduino.h>

// --- Initialization ---------------------------------------------------
// Configure all motor control pins as outputs and stop every wheel.
// Must be called from setup() before any motion command.
void motorsInit();

// --- Single-wheel control ---------------------------------------------
// Drive one specific wheel in one specific direction. Useful for fine
// manual control or for building higher-level movements not covered below.
void FR_fwd(int speed);
void FR_bck(int speed);
void FL_fwd(int speed);
void FL_bck(int speed);
void RR_fwd(int speed);
void RR_bck(int speed);
void RL_fwd(int speed);
void RL_bck(int speed);

// --- Whole-robot movement helpers -------------------------------------
// Stop every wheel immediately.
void stop_Stop();

// Drive straight forward / backward at the given speed.
void go_advance(int speed);
void go_back(int speed);

// Pivot in place. left_turn spins counter-clockwise (left wheels stop,
// right wheels go forward); right_turn does the mirror.
void left_turn(int speed);
void right_turn(int speed);

// Reverse-arc turns: the robot drives backward while curving toward the
// indicated side. Useful for backing out of a dead-end in a maze.
void left_back(int speed);
void right_back(int speed);

// Rotate in place — clockwise or counter-clockwise — at the given speed.
void clockwise(int speed);
void countclockwise(int speed);

// --- Mecanum strafing -------------------------------------------------
// Move sideways without rotating, taking advantage of the mecanum wheels.
// Each parameter is the PWM speed for one specific wheel/direction; pass
// equal values on all four for a pure side-step, or zero some out to get
// diagonal motion (see how the demo in the original sketch used these).
void right_shift(int speed_fl_fwd, int speed_rl_bck, int speed_rr_fwd, int speed_fr_bck);
void left_shift (int speed_fl_bck, int speed_rl_fwd, int speed_rr_bck, int speed_fr_fwd);

#endif // ARTEMIS_MOTORS_H
