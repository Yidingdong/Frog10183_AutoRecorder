# Universal Auto Recorder & Playback

> **Standalone library repo.** This is the dedicated home of the Universal Auto Recorder,
> maintained by **FTC Team 10183 F.R.O.G.** Everything you need is here — drop the
> `TeamCode/.../universal/` files into your own FTC project. (The source paths mirror a
> standard FTC Android-Studio project so the folder copies in 1:1.)

**Record your autonomous by driving it once. Replay it in Autonomous. Zero config.**

No motor names to edit, no path math, no odometry required — copy two files, add a few
lines to your TeleOp, and you have a working autonomous. It auto-discovers every motor and
servo on your robot and records what they do, then plays it back faithfully.

> Built for FTC SDK **11.0.0**. Pure SDK — no Pedro/Road Runner/dashboard dependency.
> MIT licensed (see `LICENSE`).

---

## Implement it in 5 steps

> Works with **any** robot — any motors, servos, and CR-servos. Only requirement: the recording
> OpMode must be a **`LinearOpMode`** (an iterative `OpMode` with `init()`/`loop()` won't work).

1. **Copy two files** into your project under `TeamCode/.../universal/`:
   `UniversalAutoRecorder.java` and `UniversalAutoPlayback.java`.
2. **Make a copy of your TeleOp** (e.g. `MyTeleOp` → `MyTeleOpRecord`) and add the recorder to the
   **copy** — leave your competition TeleOp untouched.
3. **Add the recorder to that copy** (an import, a field, and 4 calls):
   ```java
   import org.firstinspires.ftc.teamcode.universal.UniversalAutoRecorder;

   private UniversalAutoRecorder recorder;                 // field

   @Override public void runOpMode() {
       // ... your existing hardware init ...
       recorder = new UniversalAutoRecorder(hardwareMap, gamepad1, telemetry, this);
       recorder.waitForSlotSelection();                    // slot picker (before waitForStart)
       waitForStart();
       try {
           while (opModeIsActive()) {
               // ... your existing drive code, unchanged ...
               recorder.capture();                          // every loop (self-limits to 50 Hz)
           }
       } finally {
           recorder.close();                                // saves the recording
       }
   }
   ```
4. **Enable playback:** delete the `@Disabled` line in `UniversalAutoPlayback.java`.
5. **Record** (run your copy → pick a slot → press **A** → drive the autonomous by hand → **STOP**),
   then **replay** (run **"Universal Auto Playback"** → same slot → **PLAY**).

> ⚠️ **Same start, every time.** There is no localization — playback just repeats your outputs. Place
> the robot in the **exact same start position and heading**, in the **same robot state** (e.g. arm
> down, intake empty), on a **comparably charged battery** — for the recording *and* every replay.
> Any offset or difference at the start carries through the entire run.

---

## What it is — and what it isn't

It records the **outputs** your robot produced (motor power / velocity, servo positions)
at 50 Hz while you drive, and replays them on a timeline during Autonomous with smooth
interpolation. It is **not** a path follower: it has no idea where the robot actually is on
the field. That makes it incredibly fast to set up and great for many situations — and also
means it relies on a consistent start position. See **[Limitations](#limitations--when-to-use-something-else)**
before you depend on it for a precision auto.

## Features

- **Auto-discovery** — finds all `DcMotorEx` / `Servo` / `CRServo` in your `HardwareMap`. No lists to maintain.
- **Faithful to control mode** — a closed-loop **velocity** motor (e.g. a flywheel on `RUN_USING_ENCODER` + `setVelocity`) is recorded and replayed *as a velocity target* (ticks/sec, proportional to RPM), not raw power, so it holds speed on playback day.
- **Battery-voltage compensation** — open-loop power channels are scaled per-frame so a drained pack doesn't undershoot.
- **10 recording slots** with an on-Driver-Station selection UI.
- **Smooth playback** — ~200 Hz linear interpolation between recorded frames (no stair-stepping).
- **Self-describing, versioned file format** — tolerant of any legal device name (even names with spaces).
- **Crash-safe save** — recordings are written atomically (temp file + rename), so a STOP or crash mid-save never corrupts the file or destroys your previous recording in that slot.
- **Safety timeout** — playback auto-stops after the recording ends (or 30 s, whichever is later).

## How it works

```
  TeleOp (you drive)                 storage                 Autonomous (replay)
 ┌────────────────────┐        ┌──────────────────┐        ┌────────────────────┐
 │ recorder.capture() │ 50 Hz  │ slot_NN.csv      │        │ read header →      │
 │  reads motor power │ ─────► │  #FORMAT header  │ ─────► │  rebuild devices   │
 │  / velocity / servo│        │  + frame rows    │        │  interpolate ~200Hz│
 └────────────────────┘        └──────────────────┘        └────────────────────┘
```

The recorder samples every device each frame and writes a row of numbers with a timestamp.
The header is **self-describing**: it lists each device, its type, and (for motors) whether
it's a power or velocity channel — so playback rebuilds exactly the right set of devices on
any robot, and the number of columns simply adapts to your hardware.

---

## 60-second quick start

1. Copy these files into your project under `TeamCode/.../universal/`:
   `UniversalAutoRecorder.java`, `UniversalAutoPlayback.java` (and optionally
   `SampleRecorderTeleOp.java` to try it without editing your own TeleOp).
2. Add the recorder to your TeleOp — an `import`, a field, and four calls (below).
3. Build & deploy. Run your TeleOp, pick a slot, and **drive an autonomous by hand**.
4. Run **"Universal Auto Playback"** in Autonomous, pick the same slot, press play.

> **The bundled OpModes ship with `@Disabled`** (`UniversalAutoPlayback` and
> `SampleRecorderTeleOp`) so they don't clutter your OpMode list. **Delete the `@Disabled`
> line** in a file to make that OpMode appear on the Driver Station. `UniversalAutoRecorder`
> is a plain utility class — it has no annotation and is always available.

No motor names. No servo lists. Nothing robot-specific.

## Integration — recorder side (import + field + 4 calls)

```java
import org.firstinspires.ftc.teamcode.universal.UniversalAutoRecorder;

private UniversalAutoRecorder recorder;

// In runOpMode(), AFTER your subsystems/hardware set their motor modes, before waitForStart():
recorder = new UniversalAutoRecorder(hardwareMap, gamepad1, telemetry, this);
recorder.waitForSlotSelection();   // slot-picker UI on the Driver Station

// Anywhere in your main loop (safe to call every iteration — it self-rate-limits):
recorder.capture();
// Optional live status:
recorder.showTelemetry();

// After your main loop — put close() in a finally so the recording is saved even if the
// loop exits via an exception, not only a normal STOP:
//   try { while (opModeIsActive()) { ...; recorder.capture(); } }
//   finally { recorder.close(); }
recorder.close();
```

> **Tip** — Add this to a **copy** of your TeleOp (e.g. `MyTeleOpRecord`), not your competition
> TeleOp, so recording never interferes with your match code.

> **Note** — You can construct the recorder **anywhere in init**. It re-samples each motor's
> control mode the moment recording starts (your first `capture()`), so a flywheel only needs
> to be in `RUN_USING_ENCODER` by the time you press play and start driving — not earlier.

> **Warning** — Call `capture()` every loop. Don't gate it behind a button; it rate-limits
> itself to 50 Hz internally.

> **Note** — This is a **LinearOpMode** API: `waitForSlotSelection()` blocks, so an iterative
> `OpMode` (`init()`/`loop()`) can't use it as-is. Pass `this` (the 4-arg constructor) so STOP
> exits the slot picker cleanly and the Driver Station PLAY button can confirm the slot.

## Integration — playback side (nothing to edit)

`UniversalAutoPlayback` is a ready-to-run `@Autonomous` OpMode. **It ships with `@Disabled`, so
delete that annotation to make it appear on the Driver Station.** Then:

- **D-Pad Up/Down** — select a slot
- **X** — toggle battery-voltage compensation (default ON)
- **START** — begin playback

It reconstructs every device from the recording's header, restores motor directions /
zero-power behavior / velocity PIDF, and replays.

> **Warning** — Place the robot in the **same physical position and heading** it had when
> you started recording. Playback has no localization; every offset at the start carries
> through the whole run.

## Slot selection UI

```
╔══════════════════════════════════════╗
║     UNIVERSAL AUTO RECORDER         ║
╠══════════════════════════════════════╣
  Devices found: Motors:4  Servos:1  CRServos:1
  Voltage comp: ON
  Devices: M:leftFront M:rightFront V:shooter S:hood C:spinner
╠══════════════════════════════════════╣
  D-Pad UP/DOWN: Select slot
  B:             Delete slot
  A / DS PLAY:   Begin recording
╠══════════════════════════════════════╣
> Slot 1: 2026-03-15 14:30  45.2s  2260f
  Slot 2: -- EMPTY --
  ...
  Slot 10: -- EMPTY --
╚══════════════════════════════════════╝
```

`M:` = power motor, `V:` = velocity motor, `S:` = servo, `C:` = CR servo. The selected slot
is overwritten if occupied. All 10 slots are independent.

---

## File & format reference

Recordings live in the Robot Controller's app storage under `FIRST/autorecorder/`
(on most hubs this is `/sdcard/FIRST/autorecorder/`). Pull them with `adb pull` or the
OnBotJava "Manage" file browser.

```
FIRST/autorecorder/
├── index.json      ← slot metadata (date, duration, frames, device count)
├── slot_01.csv     ← recording 1
├── ...
└── slot_10.csv
```

Each CSV is a versioned, self-describing format. Metadata lines start with `#` and are
**TAB-delimited** (so device names may contain spaces); data rows are plain comma-separated
numbers:

```
#FORMAT 2
#META    created=2026-03-15 14:30:01   voltageComp=1
#DEV     idx=0   class=motor   name=leftFront   kind=power      dir=FWD   zpb=BRAKE
#DEV     idx=1   class=motor   name=shooter     kind=velocity   dir=FWD   zpb=FLOAT   pidf=1.30000,0.13000,0.00000,12.95000
#DEV     idx=2   class=servo   name=hood
#COLUMNS t_ms   vbat   d0   d1   d2
0,12.41,0.0000,0.0000,0.5000
20,12.39,0.6000,1785.4000,0.5000
```

- `t_ms` — milliseconds since recording start.
- `vbat` — battery volts that frame (present only when voltage comp was on).
- `d0…` — one value per device, in `idx` order. A **power** motor stores power (−1…1), a
  **velocity** motor stores ticks/sec, a **servo** stores position (0…1), a **CR servo**
  stores power (−1…1).

Older `#DEVICES`-style recordings from v1 still play back (all motors treated as power, no
voltage comp).

## Configuration & toggles

| Feature | Default | When to change it |
|---|---|---|
| **Velocity channels** | Auto | Automatic: any motor in `RUN_USING_ENCODER` when recording starts becomes a velocity channel. Make sure such motors are in that mode by the time you press play and drive (construction order doesn't matter). |
| **Voltage compensation** | ON (if a sensor exists) | Recorder: `recorder.setVoltageCompensation(false)` before the first `capture()`. Playback: press **X** to toggle (useful for A/B comparison). |

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| "No devices discovered" | Hardware config not loaded / wrong names | Activate the correct robot configuration on the Driver Station. |
| Robot drifts / ends in the wrong spot | Battery, start pose, or wheel slip | Align the robot the same way every time; keep voltage comp ON; re-record on a fresh pack. |
| Flywheel/shooter wrong speed on playback | Motor wasn't in `RUN_USING_ENCODER` when recording started | Make sure the flywheel is in velocity mode before you press play and drive the recording. |
| Playback does nothing | Empty slot, or rows didn't match the device count | Re-record; confirm the slot shows a size in KB. |
| "Device not found" warning | Playback robot's config differs from the recording robot | Match device names between the two configs. |
| Playback stops early | Hit the safety timeout | Timeout is `recording length + 5 s` (min 30 s); long autos are fine. |
| Arm/lift doesn't reach its positions | It's on `RUN_TO_POSITION` | Not reproduced — see Compatibility. Drive it as velocity or power, or use a hard stop. |
| A device does nothing on playback | Shown as `SKIPPED (not DcMotorEx)` on the slot screen | That motor object isn't a `DcMotorEx` and can't be recorded (very rare on modern hubs). |

## Compatibility — what works on which robot

It auto-discovers any hardware config, so it **runs** on every robot. Replay *fidelity*
depends on how each device is controlled:

**✅ Reproduced faithfully**
- Any **motor-power drivetrain** — mecanum, tank, 6-wheel, X-drive, H-drive (robot-centric).
- **Closed-loop velocity** mechanisms — a flywheel/shooter on `RUN_USING_ENCODER` + `setVelocity` (holds RPM, voltage-independent).
- **Servos** (claw, wrist, hood) and **CR-servos** (intake).
- Robots with **no voltage sensor**, **two hubs**, 20+ devices, config names with spaces, any number of I2C sensors.

**⚠️ Works, with a caveat**
- **Field-centric drive:** the record-time heading is baked into the motor outputs, so you must **start at the exact same heading** (not just position) or the whole path rotates.
- **Open-loop power arms/lifts** (`RUN_WITHOUT_ENCODER`): the *command* replays faithfully, but the resulting **position drifts** (gravity, friction, battery). Use a hard stop or a held setpoint for repeatable heights.
- **Sharing to a *different* robot:** power/servo/CR-servo channels port by name; **velocity channels also need the same motor, encoder CPR and gearing** (ticks/sec is hardware-specific).
- **Swerve:** servo-steered modules replay their commanded angles but slew uncontrolled at `t=0`; closed-loop (encoder) steering isn't reproduced.

**❌ Not reproduced**
- **`RUN_TO_POSITION` arms/lifts** — only the open-loop speed cap is captured, so the arm **won't hit its presets**. Drive such mechanisms as velocity or plain power instead (playback also warns on `RUN_TO_POSITION`).
- **A motor that changes `RunMode` mid-recording** — the channel kind is locked when recording starts.
- **Iterative `OpMode`s** — the slot picker blocks, so recording is **LinearOpMode-only** (playback is its own OpMode and is fine).

## Limitations — when to use something else

This is **output replay**, not a navigation plan. Be honest with yourself about these:

- **It replays outputs, not a position.** No odometry, no feedback. It does exactly what
  you did, every time, regardless of where the robot actually is.
- **Identical start pose is mandatory.** A few centimeters or degrees of start offset
  carries through the entire run. Use a field-wall or a start jig.
- **Battery voltage matters.** The same power gives a different speed at 13 V vs 11.5 V.
  Voltage compensation reduces this for power channels but can't push past 100% saturation,
  and the recording-day voltage still sets the baseline. Re-record on a fresh pack for best
  results.
- **Open-loop drift accumulates.** Carpet vs tile, wheel wear, robot weight — small
  differences compound over a long auto. Short, simple routines replay far more reliably.
- **No sensing, no logic.** It can't react to a missed pickup, an AprilTag, or defense.

**Use Universal Record & Playback when…** | **Use Pedro / Road Runner when…**
---|---
You want a working auto fast, with no tuning | You need centimeter-level pose accuracy
You have no odometry (dead wheels / Pinpoint) | You have odometry and time to tune
You're sharing an auto with an alliance partner | The auto must adapt to sensors / AprilTags
The routine is short and repeatable | The routine is long, precise, multi-segment

It's also a great **starting point or fallback**: ship a recorded auto now, upgrade to
path-following later — or keep one as a backup when odometry breaks at an event.

## FAQ

- **Does it need odometry?** No. It works on any drivetrain with no localization hardware.
- **Mecanum / tank / arm-bot?** Any drivetrain works. Servos and power/velocity motors
  replay faithfully; a `RUN_TO_POSITION` arm does **not** (see Compatibility).
- **Will my flywheel replay correctly?** Yes if it's a velocity channel (closed-loop in
  `RUN_USING_ENCODER`). It's reproduced via the same closed loop, so it holds RPM. It's
  still best-effort, not frame-perfect.
- **Can I share an auto with an alliance partner?** Yes — copy the two files and the slot
  CSV. Their robot must use matching device names (playback rebuilds devices by name).
- **Can I edit a recording?** It's a CSV — yes, carefully. Keep the column count consistent.

## Sharing with alliance partners

1. Give them `UniversalAutoRecorder.java` and `UniversalAutoPlayback.java`.
2. They add the recorder calls to a TeleOp (or use `SampleRecorderTeleOp`).
3. To share a *specific* auto, also copy your `slot_NN.csv` into their
   `FIRST/autorecorder/`. Device names must match between configs. (`index.json` is **not**
   required for playback — playback lists slots by file. A hand-copied slot plays fine but
   reads as `-- EMPTY --` in the *recorder* UI until that robot records over it.)

## License & credits

MIT — see `LICENSE`. Created by **FTC 10183 F.R.O.G.** Contributions and forks welcome.
