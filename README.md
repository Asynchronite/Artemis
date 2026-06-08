# Artemis — Ultrasonic Maze Robot

An autonomous maze-solving robot built on an **Arduino Mega 2560**. It feels its
way through a maze with three ultrasonic rangefinders, drives on four mecanum
wheels, and shows what it's "thinking" on a 16×2 LCD.

```
 ___   ___  ___  _   _  ___   ___   ____ ___  ____
/ _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \
| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
\___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
                 (____/
```

---

## Features

- **Three HC-SR04 ultrasonic sensors** (front / left / right — no rear) read via
  the NewPing library, with median filtering to reject noise and cross-talk.
- **Mecanum drive** — four independently-driven wheels through two MODEL-X
  H-bridge boards; turns rotate the robot **in place**, so it corners in a tight
  maze without needing extra width.
- **Discrete, predictable corner navigation** — at each corner the robot runs a
  clean `stop → pause → timed turn → pause → drive out of junction` sequence
  instead of twitchy continuous steering.
- **Selectable strategy** — most-open/greedy, right-hand wall, or left-hand wall,
  switched with a single constant.
- **Live telemetry** — the current decision (`FWD`, `STOP`, `TURNL/TURNR`,
  `EXIT`, `SPIN`) and sensor distances are shown on the LCD and streamed to the
  serial monitor.
- **Bench-safe mode** — run the full sense/decide/display loop with the motors
  held stopped, so you can verify everything on a desk without the robot driving
  off it.

---

## Hardware

| Part | Detail |
|------|--------|
| Controller | Arduino Mega 2560 |
| Drive | 4 × mecanum wheels, 2 × MODEL-X dual H-bridge boards |
| Range sensors | 3 × HC-SR04 (front, left, right) |
| Display | I²C 16×2 LCD with PCF8574 backpack |

### Pin map

All pins are defined in [`include/config.h`](include/config.h) — change wiring
there and every module picks it up.

**Motors — front pair (Right MODEL-X board)**

| Signal | Pin |
|--------|-----|
| Front-Right PWM (ENA) | 9 |
| Front-Right dir 1 / 2 | 22 / 24 |
| Front-Left dir 1 / 2 | 26 / 28 |
| Front-Left PWM (ENB) | 10 |

**Motors — rear pair (Left MODEL-X board)**

| Signal | Pin |
|--------|-----|
| Rear-Right PWM (ENA) | 11 |
| Rear-Right dir 1 / 2 | 5 / 6 |
| Rear-Left dir 1 / 2 | 7 / 8 |
| Rear-Left PWM (ENB) | 12 |

**Ultrasonic sensors** (TRIG / ECHO)

| Sensor | TRIG | ECHO |
|--------|------|------|
| Front | 47 | 46 |
| Left | 49 | 48 |
| Right | 51 | 50 |

**LCD** — I²C on the Mega's `SDA = 20`, `SCL = 21`, address `0x27` (try `0x3F`
if the screen stays blank).

---

## Software layout

The code is split into small modules so nothing hard-codes a pin or a constant:

| File | Responsibility |
|------|----------------|
| [`include/config.h`](include/config.h) | Every pin number and tunable constant |
| [`src/main.cpp`](src/main.cpp) | The maze-solving brain (sense → decide → act) |
| [`src/motors.cpp`](src/motors.cpp) | Mecanum drive primitives (`go_advance`, `clockwise`, …) |
| [`src/ultrasonic.cpp`](src/ultrasonic.cpp) | HC-SR04 distance reading via NewPing |
| [`src/display.cpp`](src/display.cpp) | Flicker-free LCD helpers |

---

## How it solves the maze

Each pass through `loop()` is a **sense → decide → act** cycle:

1. **Sense** — read the front, left and right distances (out-of-range counts as
   "fully open").
2. **Decide** — the configured strategy picks a move. In the default greedy mode
   the robot simply **drives straight down a corridor** and only turns when the
   **front is blocked**, choosing the more open side.
3. **Act** — when a corner is detected it runs a deterministic sequence:

   ```
   …FWD  →  STOP  →  (pause)  →  TURNL/TURNR  →  STOP  →  (pause)  →  EXIT  →  FWD…
   ```

   The closing `EXIT` step drives straight out of the junction so the
   rear-mounted side sensors clear the opening — otherwise the robot would see
   the corridor it just left and immediately turn back into it.

The robot never drives forward while a wall is directly ahead, so it can't knock
a wall down: if one timed turn isn't quite enough, it simply stops and turns
again.

---

## Build & upload (PlatformIO)

This is a [PlatformIO](https://platformio.org/) project (environment
`megaatmega2560`). Dependencies are declared in
[`platformio.ini`](platformio.ini) and fetched automatically:

- `marcoschwartz/LiquidCrystal_I2C`
- `teckel12/NewPing`

```bash
# Compile only
pio run

# Compile and flash the connected Mega
pio run -t upload

# Watch the serial telemetry (9600 baud)
pio device monitor -b 9600
```

> On Windows the CLI lives at
> `%USERPROFILE%\.platformio\penv\Scripts\platformio.exe` if `pio` isn't on your
> PATH. You can also use the PlatformIO toolbar in VS Code.

---

## Configuration & tuning

All knobs are in [`include/config.h`](include/config.h). The ones you'll actually
tune:

### Behaviour

| Constant | Default | What it does |
|----------|---------|--------------|
| `MAZE_STRATEGY` | `MAZE_GREEDY` | `MAZE_GREEDY` / `MAZE_RIGHT_HAND` / `MAZE_LEFT_HAND` |
| `MAZE_DRIVE_MOTORS` | `1` | `0` = bench-safe (sense & display, wheels held stopped); `1` = drive |
| `MAZE_CRUISE_SPEED` | `100` | Forward PWM speed (0–255) |
| `MAZE_TURN_SPEED` | `120` | Turning PWM speed (0–255) |

### Sensing

| Constant | Default | What it does |
|----------|---------|--------------|
| `ULTRASONIC_MAX_CM` | `400` | Max listening range; also caps ping time |
| `ULTRASONIC_PING_SAMPLES` | `3` | Pings per reading (median). Raise to reject noise/cross-talk; lower for speed |
| `FRONT_STANDOFF_CM` | `25` | Front distance at/below which "there's a wall ahead → turn" |
| `SIDE_CLEARANCE_CM` | `22` | How much side room counts as an opening worth turning into |

### Corner sequence

| Constant | Default | What it does |
|----------|---------|--------------|
| `TURN_TIME_MS` | `750` | How long the in-place spin lasts — calibrate to ~90° |
| `TURN_PAUSE_MS` | `1000` | Settle pause held before and after each turn |
| `POST_TURN_FWD_MS` | `1000` | How long to drive straight after a turn before it may turn again |

### Calibration tips

- **Turn angle** — watch one corner. If it **under**-rotates (clips the far
  wall), raise `TURN_TIME_MS`; if it **over**-rotates, lower it.
- **Turning back into the same junction** — raise `POST_TURN_FWD_MS` so it drives
  further out before looking again; lower it if it overruns a real turn that's
  just past a corner.
- **Phantom walls / false corners** — keep `ULTRASONIC_PING_SAMPLES` at `3`+ and,
  if the front trips on the angled edge of an opening, lower `FRONT_STANDOFF_CM`.

---

## Telemetry

**LCD** (top row shows the front distance and the live action, bottom row the two
side distances):

```
F:142 FWD
L:33 R:198
```

Action tags: `FWD`, `STOP`, `TURNL` / `TURNR`, `EXIT` (driving out of a junction),
`SPIN` (boxed in — rotating to find a way out).

**Serial** (9600 baud) prints the labelled distances and the chosen move each
cycle, plus the active strategy at start-up.

---

## Troubleshooting

| Symptom | Likely cause / fix |
|---------|--------------------|
| "Forward" drives **backward**, turns mirrored | Motor leads wired reversed — flip `DIR_FWD_1`/`DIR_FWD_2` in [`src/motors.cpp`](src/motors.cpp) |
| Turns rotate the wrong way | Swap the `ACT_LEFT`/`ACT_RIGHT` cases in `applyMove()` |
| Robot spins for no reason in a corridor | Cross-talk faking a phantom near wall — keep `ULTRASONIC_PING_SAMPLES` ≥ 3 |
| Under/over-rotates at corners | Calibrate `TURN_TIME_MS` |
| Immediately turns back into a junction | Increase `POST_TURN_FWD_MS` |
| LCD blank | Wrong I²C address — try `LCD_ADDR 0x3F`, or run an I²C scanner |
| Resets/browns out while driving | Give the motors their own power supply separate from the Mega's USB |
