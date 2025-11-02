# Screen Reference

The firmware renders a 16×2 character LCD. Four logical screens can be
selected (`A`, `B`, `C`, `D`). Screen `A` is the operator overview, while
Screen `B` provides paged diagnostics. Screen `D` summarises connectivity
and build information across multiple pages. Screen `C` remains reserved.
This document walks through all screens and fields so operators know how
to interpret the data.

---

## Screen A – Operator Status

```
[P]/12 D  0  ▒░
 2:05  1:30    42
```

| Field                 | Meaning                                                                |
|-----------------------|------------------------------------------------------------------------|
| `[P]` / `123`         | Brackets indicate paused; otherwise current RPM (here paused target).  |
| `12`                  | Target RPM.                                                            |
| `D  0`                | Current PWM duty (0–255).                                              |
| Gauge chars (`▒░`)    | Fill level: left = duty bar (0–8 steps), right = progress indicator.   |
| `2:05`                | Elapsed time since last reset.                                         |
| `1:30`                | Previous cycle duration (seconds).                                     |
| `42` (right-aligned)  | Keypad input buffer tail (up to 8 chars).                              |

When the motor is running the top row shows `123/70 D180 >=` (example)
with live RPM against target, and the progress/duty bars animate as the
cycle advances.

---

## Screen B – Diagnostics

Screen `B` rotates through three pages, advancing `page` whenever the
operator reselects `B`.

### Page 0 – Control Loop Snapshot


```
R070/080 D180 S>
I+05 R720 P+50
```

| Field   | Example | Meaning                                                       |
|---------|---------|---------------------------------------------------------------|
| `R070`  | `070`   | Actual RPM (signed).                                          |
| `/080`  | `080`   | Target RPM.                                                   |
| `D180`  | `180`   | PWM duty (0–255).                                             |
| `S>`    | `>`     | Motor state glyph (`I` idle, `^` ramp up, `>` run, `v` down). |
| `I+05`  | `+05`   | PID integral accumulator (±99).                               |
| `R720`  | `720`   | Rotation per stroke target in degrees.                        |
| `P+50`  | `+50`   | Net progress per cycle in degrees (signed).                   |

### Page 1 – Encoder & State Health

```
CNT+12.3k DIR>
AGE0123 ERR-04
```

| Field        | Example  | Meaning                                                           |
|--------------|----------|-------------------------------------------------------------------|
| `CNT+12.3k`  | `+12.3k` | Signed total encoder count; switches to `k` suffix past 10 000.   |
| `DIR>`       | `>`      | Current direction (`>` forward, `<` reverse).                     |
| `AGE0123`    | `0123`   | Time in the current state (milliseconds, clamped to 9999).        |
| `ERR-04`     | `-04`    | Instantaneous RPM error (target – actual).                        |

### Page 2 – Cycle History

```
C12.4s P09.8s
FW360 BK-320
```

| Field       | Example  | Meaning                                                                    |
|-------------|----------|----------------------------------------------------------------------------|
| `C12.4s`    | `12.4s`  | Duration of the most recently completed full cycle.                        |
| `P09.8s`    | `09.8s`  | Duration of the cycle before that (previous).                              |
| `FW360`     | `360`    | Degrees travelled during the last forward stroke.                          |
| `BK-320`    | `-320`   | Degrees travelled during the last reverse stroke (negative for direction). |

---

## Screen D – Connectivity & Build

Screen `D` also paginates. Re-selecting `D` cycles through three pages of
network status and firmware provenance.

### Page 0 – Wi-Fi Link

```
WiFi:StudioAP
192.168.1.42
```

| Field          | Example        | Meaning                               |
|----------------|----------------|---------------------------------------|
| `WiFi:StudioAP`| `StudioAP`     | Connected SSID (truncated to 11 chr). |
| `192.168…`     | `192.168.1.42` | Current STA IP shown without a prefix.|

### Page 1 – MQTT Broker

```
MQTT:broker.lab
Connected
```

| Field             | Example       | Meaning                                            |
|-------------------|---------------|----------------------------------------------------|
| `MQTT:broker.lab` | `broker.lab`  | Hostname/address configured for MQTT (11-char cap).|
| `Connected`       | `Connected`   | Connection state; also shows `Disconnected`.       |

### Page 2 – Firmware Build

```
Bld:2024-03-18
Time:12:34:56
```

| Field              | Example      | Meaning                                        |
|--------------------|--------------|------------------------------------------------|
| `Bld:2024-03-18`   | `2024-03-18` | Compile date straight from `__DATE__`.          |
| `Time:12:34:56`    | `12:34:56`   | Compile time straight from `__TIME__`.          |

---

## Screen Boot

Used during startup or when displaying status messages triggered by OTA.

```
WiFi ready
192.168.2.10
```

Top row shows boot status (`WiFi ready`, `MQTT reboot!`, etc.). Bottom
row can show extra info such as IP address. Messages are usually short-
lived and clear themselves once the event completes.

---

## Screen Error

Displayed when a recoverable error occurs (e.g., OTA failure).

```
OTA err  2
Auth Failed
```

Row 0 gives the error class plus numeric code, row 1 provides detail.
After a short delay the UI returns to the previous screen.

---

## Screen OTA

Shown during OTA updates.

```
Update firmware
████▒▒▒▒▒▒▒▒▒▒▒▒
```

Row 0 is the headline (`Update firmware/filesystem`). Row 1 is a 16-cell
progress bar using the custom LCD glyphs; each cell represents 12.5 % and
an individual character shows 1/8 increments.

---

## Screen C

Currently reserved for future use. The firmware clears both rows.

```
                
                
```

---

## Screen Stack Behaviour

The UI supports `pushScreen` and `popScreen`. Temporary screens like OTA
push their ID so the previous screen is restored automatically once the
task completes. Pagination (`page`) resets when switching to a different
ID and increments when the same screen is selected repeatedly (e.g.,
tapping Screen `B` cycles through diagnostic pages).
