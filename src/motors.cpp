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
//  Front Right wheel
// =====================================================================
void FR_fwd(int speed) {
    digitalWrite(RightMotorDirPin1, LOW);
    digitalWrite(RightMotorDirPin2, HIGH);
    analogWrite(speedPinR, speed);
}

void FR_bck(int speed) {
    digitalWrite(RightMotorDirPin1, HIGH);
    digitalWrite(RightMotorDirPin2, LOW);
    analogWrite(speedPinR, speed);
}

// =====================================================================
//  Front Left wheel
// =====================================================================
void FL_fwd(int speed) {
    digitalWrite(LeftMotorDirPin1, LOW);
    digitalWrite(LeftMotorDirPin2, HIGH);
    analogWrite(speedPinL, speed);
}

void FL_bck(int speed) {
    digitalWrite(LeftMotorDirPin1, HIGH);
    digitalWrite(LeftMotorDirPin2, LOW);
    analogWrite(speedPinL, speed);
}

// =====================================================================
//  Rear Right wheel
// =====================================================================
void RR_fwd(int speed) {
    digitalWrite(RightMotorDirPin1B, LOW);
    digitalWrite(RightMotorDirPin2B, HIGH);
    analogWrite(speedPinRB, speed);
}

void RR_bck(int speed) {
    digitalWrite(RightMotorDirPin1B, HIGH);
    digitalWrite(RightMotorDirPin2B, LOW);
    analogWrite(speedPinRB, speed);
}

// =====================================================================
//  Rear Left wheel
// =====================================================================
void RL_fwd(int speed) {
    digitalWrite(LeftMotorDirPin1B, LOW);
    digitalWrite(LeftMotorDirPin2B, HIGH);
    analogWrite(speedPinLB, speed);
}

void RL_bck(int speed) {
    digitalWrite(LeftMotorDirPin1B, HIGH);
    digitalWrite(LeftMotorDirPin2B, LOW);
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
