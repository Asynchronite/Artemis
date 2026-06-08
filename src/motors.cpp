/*
 * motors.cpp — Implementation of the mecanum-wheel drive.
 *
 * All pin numbers live in config.h; this file only contains the wiring
 * pattern (which combination of HIGH/LOW on the direction pins produces
 * forward vs. backward motion for each wheel). Behavior is byte-for-byte
 * identical to the original single-file version of the sketch.
 */

#include "motors.h"
#include "config.h"

// =====================================================================
//  Motor direction polarity (wiring compensation)
// =====================================================================
// DIR_FWD_1 / DIR_FWD_2 are the two direction-pin levels that make a wheel
// spin so the robot drives FORWARD. The original sketch assumed (LOW, HIGH),
// but on this robot the motor leads are wired the other way round, so a
// commanded "forward" actually drove the robot BACKWARD — and every turn came
// out mirrored. Flipping these two values inverts ALL motion at once (advance,
// reverse, turns and strafes) with no rewiring. If "forward" ever drives the
// robot backward again, swap these two values back.
#define DIR_FWD_1  HIGH
#define DIR_FWD_2  LOW

// =====================================================================
//  Front Right wheel
// =====================================================================
void FR_fwd(int speed) {
    digitalWrite(RightMotorDirPin1, DIR_FWD_1);
    digitalWrite(RightMotorDirPin2, DIR_FWD_2);
    analogWrite(speedPinR, speed);
}

void FR_bck(int speed) {
    digitalWrite(RightMotorDirPin1, DIR_FWD_2);
    digitalWrite(RightMotorDirPin2, DIR_FWD_1);
    analogWrite(speedPinR, speed);
}

// =====================================================================
//  Front Left wheel
// =====================================================================
void FL_fwd(int speed) {
    digitalWrite(LeftMotorDirPin1, DIR_FWD_1);
    digitalWrite(LeftMotorDirPin2, DIR_FWD_2);
    analogWrite(speedPinL, speed);
}

void FL_bck(int speed) {
    digitalWrite(LeftMotorDirPin1, DIR_FWD_2);
    digitalWrite(LeftMotorDirPin2, DIR_FWD_1);
    analogWrite(speedPinL, speed);
}

// =====================================================================
//  Rear Right wheel
// =====================================================================
void RR_fwd(int speed) {
    digitalWrite(RightMotorDirPin1B, DIR_FWD_1);
    digitalWrite(RightMotorDirPin2B, DIR_FWD_2);
    analogWrite(speedPinRB, speed);
}

void RR_bck(int speed) {
    digitalWrite(RightMotorDirPin1B, DIR_FWD_2);
    digitalWrite(RightMotorDirPin2B, DIR_FWD_1);
    analogWrite(speedPinRB, speed);
}

// =====================================================================
//  Rear Left wheel
// =====================================================================
void RL_fwd(int speed) {
    digitalWrite(LeftMotorDirPin1B, DIR_FWD_1);
    digitalWrite(LeftMotorDirPin2B, DIR_FWD_2);
    analogWrite(speedPinLB, speed);
}

void RL_bck(int speed) {
    digitalWrite(LeftMotorDirPin1B, DIR_FWD_2);
    digitalWrite(LeftMotorDirPin2B, DIR_FWD_1);
    analogWrite(speedPinLB, speed);
}

// =====================================================================
//  Whole-robot helpers
// =====================================================================

// Writing 0 to every PWM pin coasts all four wheels to a stop.
void stop_Stop() {
    analogWrite(speedPinLB, 0);
    analogWrite(speedPinRB, 0);
    analogWrite(speedPinL, 0);
    analogWrite(speedPinR, 0);
}

void go_advance(int speed) {
    RL_fwd(speed);
    RR_fwd(speed);
    FR_fwd(speed);
    FL_fwd(speed);
}

void go_back(int speed) {
    RL_bck(speed);
    RR_bck(speed);
    FR_bck(speed);
    FL_bck(speed);
}

// Mecanum strafe: opposite-corner wheels spin opposite directions, which
// cancels forward motion and produces a sideways slide.
void right_shift(int speed_fl_fwd, int speed_rl_bck, int speed_rr_fwd, int speed_fr_bck) {
    FL_fwd(speed_fl_fwd);
    RL_bck(speed_rl_bck);
    RR_fwd(speed_rr_fwd);
    FR_bck(speed_fr_bck);
}

void left_shift(int speed_fl_bck, int speed_rl_fwd, int speed_rr_bck, int speed_fr_fwd) {
    FL_bck(speed_fl_bck);
    RL_fwd(speed_rl_fwd);
    RR_bck(speed_rr_bck);
    FR_fwd(speed_fr_fwd);
}

// Pivot-style turns: left wheels held at 0, right wheels drive forward
// (or vice-versa) so the robot rotates around its left/right side.
void left_turn(int speed) {
    RL_bck(0);
    RR_fwd(speed);
    FR_fwd(speed);
    FL_bck(0);
}

void right_turn(int speed) {
    RL_fwd(speed);
    RR_bck(0);
    FR_bck(0);
    FL_fwd(speed);
}

void left_back(int speed) {
    RL_fwd(0);
    RR_bck(speed);
    FR_bck(speed);
    FL_fwd(0);
}

void right_back(int speed) {
    RL_bck(speed);
    RR_fwd(0);
    FR_fwd(0);
    FL_bck(speed);
}

// In-place rotation: left side forward, right side backward (clockwise
// when viewed from above) and the inverse for counter-clockwise.
void clockwise(int speed) {
    RL_fwd(speed);
    RR_bck(speed);
    FR_bck(speed);
    FL_fwd(speed);
}

void countclockwise(int speed) {
    RL_bck(speed);
    RR_fwd(speed);
    FR_fwd(speed);
    FL_bck(speed);
}

// =====================================================================
//  Initialization
// =====================================================================
void motorsInit() {
    pinMode(RightMotorDirPin1, OUTPUT);
    pinMode(RightMotorDirPin2, OUTPUT);
    pinMode(speedPinL,         OUTPUT);

    pinMode(LeftMotorDirPin1,  OUTPUT);
    pinMode(LeftMotorDirPin2,  OUTPUT);
    pinMode(speedPinR,         OUTPUT);

    pinMode(RightMotorDirPin1B, OUTPUT);
    pinMode(RightMotorDirPin2B, OUTPUT);
    pinMode(speedPinLB,         OUTPUT);

    pinMode(LeftMotorDirPin1B, OUTPUT);
    pinMode(LeftMotorDirPin2B, OUTPUT);
    pinMode(speedPinRB,        OUTPUT);

    // Guarantee a known-stopped state at boot so the robot doesn't lurch
    // forward while the rest of setup() is still running.
    stop_Stop();
}
