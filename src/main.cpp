/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____
 * / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \
 *| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
 * \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
 *                  (____/
 *
 * Artemis — ultrasonic maze robot.
 *   Board:   Arduino Mega 2560
 *   Drive:   4 mecanum wheels via two MODEL-X H-bridge boards
 *   Sensors: 3x HC-SR04 ultrasonic rangefinders — front, left, right
 *            (no rear sensor), all driven via the NewPing library
 *   Display: I2C 16x2 LCD (PCF8574 backpack)
 *
 * Each subsystem lives in its own module:
 *   config.h     — every pin number and tunable constant
 *   motors.*     — drive control (go_advance, left_turn, ...)
 *   ultrasonic.* — HC-SR04 distance reading (readDistance(sensor))
 *   display.*    — non-flickering LCD helpers (lcdPrintLine, lcdPrintAt)
 *
 * This file wires everything together and runs the maze-solving brain inside
 * loop(): a sense -> decide -> act cycle whose behaviour is picked by
 * MAZE_STRATEGY in config.h (most-open/greedy by default, or either-hand
 * wall-following).
 */

#include <Arduino.h>
#include "config.h"
#include "motors.h"
#include "display.h"
#include "ultrasonic.h"

// The move the brain can decide on each loop. Pure pivots/spin keep the
// reactive control simple: turn a little, look again, repeat.
enum MazeAction {
    ACT_FORWARD,   // path ahead is clear — drive straight
    ACT_LEFT,      // rotate in place toward the left
    ACT_RIGHT,     // rotate in place toward the right
    ACT_SPIN,      // boxed in (no rear sensor) — rotate in place to escape
    ACT_STOP       // hold still (used for the pauses either side of a turn)
};

// Driving phases used by loop() to take a corner as a clean, discrete sequence:
// run normally, pause, turn for a fixed time, pause again, then resume.
enum DrivePhase { PHASE_RUN, PHASE_PAUSE_PRE, PHASE_TURN, PHASE_PAUSE_POST, PHASE_ADVANCE };

// Print one labelled distance reading to the serial monitor.
static void reportReading(const __FlashStringHelper* label, const UltrasonicReading& r) {
    Serial.print(label);
    if (!r.inRange) {
        Serial.println(F(": out of range / no echo"));
    } else {
        Serial.print(F(": "));
        Serial.print(r.cm);
        Serial.print(F(" cm ("));
        Serial.print(r.inches);
        Serial.println(F(" in)"));
    }
}

// Format a single distance for the (cramped) LCD: the centimetre value, or
// "--" when the sensor sees nothing within range.
static String lcdCm(const UltrasonicReading& r) {
    return r.inRange ? String(r.cm) : String("--");
}

// How much room there is in a sensor's direction, in centimetres. Out of range
// means "nothing within ULTRASONIC_MAX_CM" — i.e. the MOST open direction — so
// it maps to the maximum, NOT to the 0 that .cm reports on a timeout (which
// would read as "wall right here" and send the robot the wrong way).
static int roomCm(const UltrasonicReading& r) {
    return r.inRange ? (int)r.cm : ULTRASONIC_MAX_CM;
}

// The brain. Given the room available ahead and to each side, pick a move
// according to the strategy selected by MAZE_STRATEGY in config.h.
static MazeAction decideMove(int frontRoom, int leftRoom, int rightRoom) {
    bool frontBlocked = frontRoom <= FRONT_STANDOFF_CM;

#if MAZE_STRATEGY == MAZE_GREEDY
    // Drive straight down the corridor until something is actually in the way.
    // We deliberately do NOT veer toward a side opening while the path ahead is
    // clear — that just twitches the robot at every doorway. Side openings only
    // matter once the front is blocked (a corner / dead-end), handled below.
    if (!frontBlocked) {
        return ACT_FORWARD;
    }
    // Wall ahead: pivot toward the roomier side, or spin if walled in on three
    // sides (there's no rear sensor, so we can't simply back up sighted).
    if (rightRoom >= leftRoom && rightRoom > SIDE_CLEARANCE_CM) return ACT_RIGHT;
    if (leftRoom > SIDE_CLEARANCE_CM)                          return ACT_LEFT;
    return ACT_SPIN;

#elif MAZE_STRATEGY == MAZE_RIGHT_HAND
    // Right-hand rule: take the right opening if there is one, else go straight,
    // else the left opening, else turn around. Solves any simply-connected maze.
    if (rightRoom > SIDE_CLEARANCE_CM) return ACT_RIGHT;
    if (!frontBlocked)                 return ACT_FORWARD;
    if (leftRoom > SIDE_CLEARANCE_CM)  return ACT_LEFT;
    return ACT_SPIN;

#else // MAZE_LEFT_HAND
    // Left-hand rule: the mirror image of the right-hand rule.
    if (leftRoom > SIDE_CLEARANCE_CM)  return ACT_LEFT;
    if (!frontBlocked)                 return ACT_FORWARD;
    if (rightRoom > SIDE_CLEARANCE_CM) return ACT_RIGHT;
    return ACT_SPIN;
#endif
}

// Translate a decision into motor commands (helpers from motors.h).
static void applyMove(MazeAction action) {
    switch (action) {
        case ACT_FORWARD: go_advance(MAZE_CRUISE_SPEED);   break;
        // Turns ROTATE IN PLACE (counter-/clockwise) rather than pivoting
        // around one side, so cornering needs no extra width — important in a
        // tight maze. countclockwise() swings the nose left, clockwise() right.
        case ACT_LEFT:    countclockwise(MAZE_TURN_SPEED); break;
        case ACT_RIGHT:   clockwise(MAZE_TURN_SPEED);      break;
        case ACT_SPIN:    clockwise(MAZE_TURN_SPEED);      break;
        case ACT_STOP:    stop_Stop();                     break;
    }
}

// Short tag for the LCD / serial monitor so you can SEE what the brain decided.
static const char* actionLabel(MazeAction action) {
    switch (action) {
        case ACT_FORWARD: return "FWD";
        case ACT_LEFT:    return "LEFT";
        case ACT_RIGHT:   return "RIGHT";
        case ACT_SPIN:    return "SPIN";
        case ACT_STOP:    return "STOP";
    }
    return "?";
}

void setup() {
    Serial.begin(9600);

    motorsInit();
    displayInit();

    lcdPrintLine(0, "Artemis ready");
    lcdPrintLine(1, "v0.3 maze");

    Serial.print(F("Maze strategy: "));
#if MAZE_STRATEGY == MAZE_GREEDY
    Serial.println(F("most-open / greedy"));
#elif MAZE_STRATEGY == MAZE_RIGHT_HAND
    Serial.println(F("right-hand wall"));
#else
    Serial.println(F("left-hand wall"));
#endif
}

void loop() {
    // 1. SENSE — read the three rangefinders (front, left, right; no rear).
    UltrasonicReading front = readDistance(US_FRONT);
    UltrasonicReading left  = readDistance(US_LEFT);
    UltrasonicReading right = readDistance(US_RIGHT);

    int frontRoom = roomCm(front);
    int leftRoom  = roomCm(left);
    int rightRoom = roomCm(right);

    // 2. DECIDE — natural strategy choice for the current sensor picture.
    MazeAction natural = decideMove(frontRoom, leftRoom, rightRoom);
    bool frontBlocked  = (frontRoom <= FRONT_STANDOFF_CM);
    bool corneringTurn = (natural == ACT_LEFT || natural == ACT_RIGHT) && frontBlocked;
    MazeAction action  = natural;

    // 2b. CORNER HANDLING — a little state machine so corners come out clean
    //     even with rear-mounted side sensors and no wheel encoders:
    //       PHASE_RUN   : follow the strategy reactively.
    //       PHASE_CLEAR : at a corner, creep STRAIGHT for up to CORNER_CLEAR_MS
    //                     (or until a front wall is CORNER_CLEAR_STOP_CM away) so
    //                     the body clears the corner before turning.
    //       PHASE_TURN  : commit to spinning in place for TURN_COMMIT_MS (~90°)
    //                     so it can't half-turn, read the diagonal as "clear",
    //                     and drive into the wall.
    //     A gentle veer (a side opens while the front is still clear) is NOT a
    //     corner, so it stays reactive and skips all of this.
    static DrivePhase phase = PHASE_RUN;
    static MazeAction committedTurn = ACT_FORWARD;
    static unsigned long phaseStart = 0;
    unsigned long now = millis();

    switch (phase) {
        case PHASE_RUN:
            if (corneringTurn) {
                committedTurn = natural;            // remember which way to turn
                phase = PHASE_PAUSE_PRE;            // ...but stop and settle first
                phaseStart = now;
                action = ACT_STOP;
            }
            // else: action stays the natural decision (forward / veer / spin)
            break;

        case PHASE_PAUSE_PRE:
            if (now - phaseStart < TURN_PAUSE_MS) {
                action = ACT_STOP;                  // hold still before turning
            } else {
                phase = PHASE_TURN;
                phaseStart = now;
                action = committedTurn;
            }
            break;

        case PHASE_TURN:
            if (now - phaseStart < TURN_TIME_MS) {
                action = committedTurn;             // spin in place for a fixed time
            } else {
                phase = PHASE_PAUSE_POST;
                phaseStart = now;
                action = ACT_STOP;
            }
            break;

        case PHASE_PAUSE_POST:
            if (now - phaseStart < TURN_PAUSE_MS) {
                action = ACT_STOP;                  // settle after turning
            } else {
                phase = PHASE_ADVANCE;              // pull forward out of the junction
                phaseStart = now;
                action = ACT_FORWARD;
            }
            break;

        case PHASE_ADVANCE:
            // Drive straight out of the junction so the (rear-mounted) side
            // sensors clear the opening we just turned out of — otherwise we'd
            // immediately turn back into it. Stop early if a wall comes up ahead.
            if (now - phaseStart < POST_TURN_FWD_MS && !frontBlocked) {
                action = ACT_FORWARD;
            } else {
                phase = PHASE_RUN;                  // clear of the junction -> follow normally
                action = natural;
            }
            break;
    }

    // 3. ACT — drive the motors accordingly. When MAZE_DRIVE_MOTORS is 0 we
    //    still sense, decide and display, but keep the wheels stopped so the
    //    robot can be tested safely on a bench.
#if MAZE_DRIVE_MOTORS
    applyMove(action);
#else
    stop_Stop();
#endif

    // 4. REPORT — LCD shows the front distance + current action on top and the
    //    two side distances below (rows overwritten in place, no flicker); full
    //    detail plus the decision goes to serial. The action reads "CRNR" while
    //    creeping to clear a corner and "TURNL"/"TURNR" during a committed turn.
    const char* label = actionLabel(action);
    if (phase == PHASE_TURN)          label = (committedTurn == ACT_LEFT) ? "TURNL" : "TURNR";
    else if (phase == PHASE_ADVANCE)  label = "EXIT";
    lcdPrintLine(0, String("F:") + lcdCm(front) + " " + label);
    lcdPrintLine(1, String("L:") + lcdCm(left) + " R:" + lcdCm(right));

    reportReading(F("Front"), front);
    reportReading(F("Left"),  left);
    reportReading(F("Right"), right);
    Serial.print(F("-> "));
    Serial.println(label);

    // 5. Brief idle so the LCD/sensors aren't hammered. Lower this for snappier
    //    reactions once you're happy with the behaviour. (NewPing also enforces
    //    its own short minimum gap between pings on each sensor.)
    delay(50);
}
