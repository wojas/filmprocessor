# Repository Guidelines

## Project Structure & Module Organization
- `src/` holds the core firmware (`main.cpp`, `motor.cpp`, MQTT helpers); keep hardware-facing logic here.
- `include/` stores shared headers and private config such as `secrets.h` (copied from `secrets.h.example`).
- `lib/` is reserved for custom reusable libraries, while `doc/` carries schematics and reference photos.
- `helpers/` and `tools/` contain Python utilities for log analysis and motor tuning; keep experimental scripts out of `src/`.
- `test/` follows PlatformIO’s layout for Unity tests; `tmp/` is for local logs and should stay unversioned.

## Build, Test, and Development Commands
- `pio run` builds the default `nodemcu-32s` environment with C++17 flags.
- `pio run -t upload` flashes firmware over USB; `pio device monitor -b 115200` opens the serial console with the configured filters.
- `pio run -t upload --upload-port <ip>` enables OTA once `platformio.local.ini` defines `upload_port`.
- `pio test -e nodemcu-32s` runs unit tests under Unity; guard hardware-dependent cases with `#ifdef`s to keep CI friendly.

## Coding Style & Naming Conventions
- Use four-space indentation, brace-on-same-line (1TBS) formatting, and keep files clang-format friendly.
- Prefer `snake_case` for functions and globals (`motor_calc_rpm`), `PascalCase` for classes, and all-caps for `#define` macros.
- Leverage C++17 features already enabled (e.g., `auto`, `constexpr`, structured bindings) and route diagnostics through `LOGF`/`LOG` helpers.
- Place hardware constants near their usage in `*.cpp` files; expose only the minimal API in headers.
- Words in names that are initialisms or acronyms (e.g. “URL” or “NATO”) have a consistent case.
- Add a short concise doc comment before each function, method, or class.

## Testing Guidelines
- Add Unity test cases under `test/` named `test_<feature>.cpp`, mirroring the module under test.
- Stub Wi-Fi/MQTT dependencies when possible; reserve encoder or motor integration checks for on-device runs and note the hardware required.
- Capture before/after serial output or log excerpts in `doc/` when validating changes to timing or motor control.

## Commit & Pull Request Guidelines
- Follow the `area: concise summary` convention evident in history (`motor: flush buffered metrics on pause`).
- Keep subject lines in imperative mood, list hardware impacts in the body, and link related issues or TODOs.
- For pull requests, describe the test matrix (USB, OTA, bench runs), include photos or logs when behavior changes, and call out config prerequisites.
- Limit every commit message line to 72 characters and escape newlines properly when invoking `git commit` via the shell.
- Limit every commit message line to 72 characters, and when invoking `git commit` avoid any `\n` escapes. Pass each paragraph as its own `-m` flag so line wrapping stays explicit.

## Configuration & Security Notes
- Never commit populated `include/secrets.h` or `platformio.local.ini`; share sample values through the provided `*.example` files instead.
- Document any new credentials, brokers, or pin mappings in `doc/` so builders can reproduce your setup safely.
