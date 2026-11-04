# sysid_analyze.py

WPILib SysId-style feedforward characterization from a raw CTRE Phoenix 6
`.hoot` log, or directly from an already-converted `.wpilog` (e.g. one you
already have open in AdvantageScope) — no Pro license, no Tuner X GUI needed.

## What it figures out automatically

- **Which mechanism was characterized** — it looks for a custom string
  signal in the log whose values are exactly the WPILib `SysIdRoutine`
  state vocabulary (`quasistatic-forward`, `quasistatic-reverse`,
  `dynamic-forward`, `dynamic-reverse`, `none`). The signal's name tells you
  which test it was: CTRE's swerve generator writes
  `SysIdTranslation_State` / `SysIdSteer_State` / `SysIdRotation_State`, but
  any mechanism that logs its `SysIdRoutine.State` via
  `SignalLogger.writeString(...)` is recognized the same way.
- **Which TalonFX devices are the drive wheels** — it clusters the TalonFX
  devices on the CAN bus by velocity variance during the test. The 4 that
  actually spin are "active"; the 4 that stay still are "quiescent". For a
  Translation or Rotation test the active group is the drive motors; for a
  Steer test it's the steer motors. No CAN-ID numbering is assumed.

It then fits `V = kS*sign(v) + kV*v + kA*a` by least squares over the
quasistatic + dynamic phases, pooling samples from every motor in the
active group, and reports kS/kV/kA and R².

If it can find a Tuner X `TunerConstants.java` (default: searched under
`--project-dir`, which defaults to the current directory), it also converts
kV/kA into output-shaft and, for drive motors, linear (m/s) units using the
gear ratio and wheel radius from that file.

## Prerequisites

- Python 3.10+.
- **owlet, only if you're passing a `.hoot` log** (a `.wpilog` is read
  directly and doesn't need it). owlet is CTRE's `.hoot` → `.wpilog`
  converter, the same one Tuner X uses internally. Get it one of these ways:
  - It ships with Phoenix Tuner X; add Tuner X's install directory to PATH.
  - Download a standalone build for your platform from CTRE's
    redistribution index: <https://redist.ctr-electronics.com/index.json>
  - Point this tool at an existing copy with `--owlet /path/to/owlet`.

  If owlet isn't found, the tool prints these same instructions and exits
  rather than guessing or downloading anything on its own.

  **Match owlet to the hoot log, not to the Java vendordep.** A hoot log's
  format is set by the Phoenix 6 *firmware* on the roboRIO/CANivore, which
  is flashed separately from (and can lag) the `Phoenix6-*.json` vendordep
  version this repo builds against. If conversion fails with `Input file is
  an unsupported version: N`, run `owlet --compliancy your.hoot` and compare
  its `owlet:` number against `hoot log:` — on a mismatch, try an older
  owlet build from the redistribution index above until the two agree.

## Setup

```bash
cd tools/sysid
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## Usage

```bash
# from the repo root
tools/sysid/.venv/bin/python3 tools/sysid/sysid_analyze.py \
  /path/to/log.hoot --plot out.png

# a .wpilog (e.g. exported from, or already open in, AdvantageScope) skips
# the owlet conversion step entirely
tools/sysid/.venv/bin/python3 tools/sysid/sysid_analyze.py \
  /path/to/log.wpilog --plot out.png
```

Pass `--plot imshow` to pop up an interactive matplotlib window instead of
saving a file (needs a display).

Run with `--help` for all options (gear ratio / wheel radius overrides,
trim/threshold tuning, keeping the intermediate `.wpilog`, etc).

## Method notes

- Velocity/acceleration/voltage come straight from each TalonFX's own
  `Velocity`/`MotorVoltage` status signals (rotor rotations), not from the
  robot's odometry.
- Acceleration is the numerical gradient of velocity within each
  contiguous test phase (gaps between phases, where the mechanism isn't
  being driven, are excluded).
- The first `--trim-s` seconds of each phase are dropped to avoid state-
  transition transients; quasistatic samples below `--quasi-vel-threshold`
  rotor rot/s are dropped so pre-breakaway static-friction noise doesn't
  bias kV.
- This mirrors the core of WPILib SysId's "simple/drivetrain" feedforward
  analysis; it does not replicate every internal filtering heuristic of the
  official SysId application, so treat the result as a solid estimate to
  sanity-check against (or feed into) that tool rather than a byte-for-byte
  reproduction.
