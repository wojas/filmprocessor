# Keypad Reference

The 4×4 membrane keypad is mapped via the pattern-matching input router.
Routes are defined in `init_keypad_routes()` and use short patterns to
adjust motor settings or trigger actions. This document explains every
registered pattern, what value ranges are accepted, and how the motor
responds.

## Pattern Syntax

- `A`, `B`, `C`, `D`, `#`, and literal digits (`0–9`) are matched exactly.
- `n` placeholders represent digits captured as a number (one group per
  pattern).
- `*` clears the buffer when it is not empty; otherwise it is treated as
  a literal `*`.

Example: `Bnnn#` means “press `B`, up to three digits, then `#`”. The
numeric portion is exposed to handlers as `res.number`; the handler also
tracks whether the first digit was a `0` via `res.leading_zero` (used to
toggle sign behaviour).

## Registered Routes

### Preset Keys `1–7`
These keys load predefined rotation/RPM/progress combinations:

| Key | Rotation (deg) | Progress (deg) | RPM | Notes                                   |
|-----|----------------|----------------|-----|-----------------------------------------|
| `1` | 900            | 50             | 70  | Recommended for 1500 and 2500 tanks.    |
| `2` | 720            | 50             | 50  | Recommended for 3000 series tanks.      |
| `3` | 360            | 50             | 25  | Gentle motion for washing, if desired.  |
| `4` | 720            | 50             | 60  | In between the two normal speeds.       |
| `5` | 360            | 50             | 5   | Slow crawl for inspection.              |
| `6` | 360            | 50             | 10  | Slightly faster inspection mode.        |
| `7` | 900            | 50             | —   | Applies duty `230` (manual high-torque) | 

Each preset first clears any custom duty, then sets rotation and progress
before requesting the RPM/duty that suits the scenario. Press `0` or `#`
to halt or reset once finished.

### `0` – Stop (Zero Duty)
- **Input**: Single `0`.
- **Behaviour**: Sets PWM duty to `0`, effectively stopping the motor and
  pausing the loop.

### `#` – Reset Timer
- **Input**: Single `#`.
- **Behaviour**: Calls `reset_timer()`. The elapsed time display on Screen
  `A` starts over and the previous cycle duration is updated if the last
  run exceeded 15 s (short presses are ignored to avoid spurious data).

### `A` – Show Screen A
- **Input**: Single `A`.
- **Behaviour**: Switches the LCD to Screen `A` (operator overview).

### `B` – Show Screen B
- **Input**: Single `B`.
- **Behaviour**: Switches the LCD to Screen `B`. Repeated taps cycle
  through the diagnostic sub-pages.

### `D` – Show Screen D
- **Input**: Single `D`.
- **Behaviour**: Switches to Screen `D`. Each tap advances through the
  connectivity/build pages so you can confirm Wi-Fi, MQTT, and firmware
  metadata without leaving the keypad.

## Advanced Settings

### `*Annn#` – Set Target RPM
- **Input**: `*`, `A`, digits (`0–99`), `#`.
- **Behaviour**: Updates the closed-loop target RPM. Values above 99 are
  ignored to prevent accidental high-speed requests.

### `*Bnnn#` – Set Progress Bias
- **Input**: `*`, `B`, digits, `#`.
- **Special rule**: If the numeric sequence starts with `0`, the value is
  treated as negative (e.g., `B012#` ⇒ `-12`).
- **Behaviour**: Adjusts `progressDegrees`, i.e., net degrees advanced per
  complete cycle beyond the nominal forward/backward rotation. Positive
  values bias forward motion, negative values bias backward motion.

### `*Cnnn#` – Set Rotation Per Cycle
- **Input**: `*`, `C`, digits, `#`.
- **Behaviour**: Updates the forward/backward stroke size in degrees. The
  control loop recalculates stroke targets immediately; existing cycles may
  complete using the previous value before the change takes full effect.

### `*Dnnn#` – Set PWM Duty
- **Input**: `*`, `D`, digits (`0–255`), `#`.
- **Behaviour**: Switches the controller into open-loop duty mode with the
  specified raw PWM value. Useful for manual testing. Values outside the
  `0–255` range are clamped.

## Buffer Handling

- Digits accumulate until a handler matches; the buffer is cleared once a
  route fires.
- Pressing `*` midway clears the buffer so you can re-enter a command.
- Buffer contents appear on Screen `A` (lower-right) so you can verify the
  digits before hitting `#`.

## Interaction Tips

- Use `#` before a new numeric command to reset the timer for every
  new development step.
- Use `*` to cancel existing input.

## Expert Tips    

- For incremental progress tweaks, `B050#` (forward bias) and `B012#`
  (reverse bias) let you fine-tune without changing presets.
- When experimenting with duty (`Dnnn#`), monitor Screen `B` page 1 for
  RPM error to see how far you are slipping from the target.
