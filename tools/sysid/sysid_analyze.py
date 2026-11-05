#!/usr/bin/env python3
"""
sysid_analyze.py - WPILib SysId-style feedforward characterization from a CTRE
Phoenix 6 ".hoot" log (the native roboRIO signal-logger format) or an
already-converted ".wpilog" (e.g. one you already have open in
AdvantageScope; no owlet needed for this path).

What it does, automatically, from the log file alone:
  1. If given a .hoot log, converts it to .wpilog using CTR Electronics'
     "owlet" CLI tool. owlet is a prerequisite for this path -- see the
     README -- and is looked up on PATH or at --owlet. A .wpilog is read
     directly, skipping this step.
  2. Scans the log for a custom string signal whose values are the
     standard WPILib SysIdRoutine state vocabulary
     (quasistatic-forward/-reverse, dynamic-forward/-reverse, none). The
     signal's name identifies *which* mechanism was characterized -- CTRE's
     swerve generator names these "SysIdTranslation_State" (drive motors),
     "SysIdSteer_State" (steer motors) or "SysIdRotation_State" (drive
     motors, rotation-in-place), but any Phoenix6 SignalLogger.writeString
     name following the WPILib SysIdRoutine convention is recognized.
  3. For swerve translation/rotation tests, clusters the TalonFX devices on
     the bus into two groups by velocity variance during the test: the
     4 motors that actually spin are the drive wheels, the quiescent ones
     are the steer motors (and vice-versa for a steer test). This is how
     "the drive wheels" are found -- no CAN-ID assumptions, no project
     source required.
  4. Fits the standard SysId feedforward model V = kS*sign(v) + kV*v + kA*a
     in two stages, pooling data from all motors in the identified group:
       - kS, kV: 2-parameter fit (V = kS*sign(v) + kV*v) over the
         quasistatic phases only, where acceleration is assumed negligible.
       - kA: the quasistatic kS/kV prediction is subtracted from voltage,
         and the residual is regressed against acceleration over the
         dynamic phases only.
     Velocity is resampled onto a uniform time grid (one matching each
     phase's own median sample interval) before differentiating to get
     acceleration, since raw CAN bus timestamps aren't perfectly evenly
     spaced.

Optionally (best-effort, not required): if a CTRE Tuner X "TunerConstants"
Java file can be found (via --project-dir, default: current directory),
drive-motor gear ratio and wheel radius are read from it so kV/kA can be
reported in real units (m/s, m/s^2) instead of motor rotations.

Usage:
    python3 sysid_analyze.py /path/to/log.hoot [options]
    python3 sysid_analyze.py /path/to/log.wpilog [options]

See --help for all options.
"""
from __future__ import annotations

import argparse
import glob
import os
import re
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field

import numpy as np

SYSID_STATE_VALUES = {
    "quasistatic-forward",
    "quasistatic-reverse",
    "dynamic-forward",
    "dynamic-reverse",
    "none",
}

OWLET_INDEX_URL = "https://redist.ctr-electronics.com/index.json"

OWLET_INSTALL_HELP = (
    "owlet (CTRE's .hoot -> .wpilog converter) was not found on PATH.\n"
    "Install it one of these ways:\n"
    "  - It ships with Phoenix Tuner X; add Tuner X's install directory to PATH.\n"
    f"  - Download a standalone build for your platform from CTRE's redistribution index:\n"
    f"    {OWLET_INDEX_URL}\n"
    "  - Point this tool at an existing copy with --owlet /path/to/owlet."
)

DEVICE_SIGNAL_RE = re.compile(r"^Phoenix6/([A-Za-z0-9]+)-(\d+)/(.+)$")

# Friendly labels + which motor group the fit should use, for CTRE's
# swerve-generator default SignalLogger names.
KNOWN_SWERVE_TESTS = {
    "sysidtranslation": ("Swerve Drivetrain - Translation (drive motors)", "drive"),
    "sysidsteer": ("Swerve Drivetrain - Steer (steer motors)", "steer"),
    "sysidrotation": ("Swerve Drivetrain - Rotation (drive motors)", "drive"),
}


# --------------------------------------------------------------------------
# owlet (hoot -> wpilog converter) discovery
# --------------------------------------------------------------------------

def find_owlet(explicit_path: str | None) -> str:
    """Locates the owlet binary: explicit_path if given, else the first match on PATH.

    Raises:
        FileNotFoundError: When explicit_path is given but doesn't exist.
        RuntimeError: When explicit_path is omitted and owlet isn't on PATH.
    """
    if explicit_path:
        if not os.path.isfile(explicit_path):
            raise FileNotFoundError(f"--owlet path does not exist: {explicit_path}")
        return explicit_path

    on_path = shutil.which("owlet")
    if on_path:
        return on_path

    raise RuntimeError(OWLET_INSTALL_HELP)


def convert_hoot_to_wpilog(
    owlet_path: str, hoot_path: str, out_path: str, enable_buffer_s: float = 1.0, quiet: bool = False
) -> None:
    """Converts a raw Phoenix6 .hoot log to .wpilog at out_path, via the owlet CLI.

    Raises:
        RuntimeError: When owlet exits non-zero or doesn't produce out_path.
    """
    cmd = [owlet_path, "-f", "wpilog", "-e", str(enable_buffer_s),
           "--unlicensed", hoot_path, out_path]
    if not quiet:
        print("Running:", " ".join(cmd), file=sys.stderr)
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0 or not os.path.isfile(out_path):
        raise RuntimeError(
            f"owlet conversion failed (exit {result.returncode}):\n"
            f"stdout: {result.stdout}\nstderr: {result.stderr}"
        )


# --------------------------------------------------------------------------
# WPILOG parsing
# --------------------------------------------------------------------------

@dataclass
class LogSignal:
    """One WPILOG entry's full time series.

    Attributes:
        name: Entry name, e.g. "Phoenix6/TalonFX-21/Velocity".
        type: WPILOG type string ("double", "string", "int64", "boolean" or "float").
        t: Timestamps, s, one per sample.
        v: Sample values; a numpy array for numeric types, a list of str for "string".
    """
    name: str
    type: str
    t: np.ndarray = field(default_factory=lambda: np.array([]))
    v: np.ndarray | list = field(default_factory=list)


def read_wpilog(path: str) -> dict[str, LogSignal]:
    """Reads every entry of a .wpilog file into a LogSignal, keyed by entry name."""
    from wpiutil.log import DataLogReader

    reader = DataLogReader(path)
    if not reader.isValid():
        raise RuntimeError(f"{path} is not a valid WPILOG file")

    entries: dict[int, LogSignal] = {}
    ts_buf: dict[int, list] = {}
    val_buf: dict[int, list] = {}

    for rec in reader:
        if rec.isStart():
            d = rec.getStartData()
            entries[d.entry] = LogSignal(name=d.name, type=d.type)
            ts_buf[d.entry] = []
            val_buf[d.entry] = []
        elif rec.isControl():
            continue
        else:
            eid = rec.getEntry()
            sig = entries.get(eid)
            if sig is None:
                continue
            t_sec = rec.getTimestamp() / 1_000_000.0
            if sig.type == "double":
                val = rec.getDouble()
            elif sig.type == "string":
                val = rec.getString()
            elif sig.type == "int64":
                val = rec.getInteger()
            elif sig.type == "boolean":
                val = rec.getBoolean()
            elif sig.type == "float":
                val = rec.getFloat()
            else:
                continue
            ts_buf[eid].append(t_sec)
            val_buf[eid].append(val)

    out: dict[str, LogSignal] = {}
    for eid, sig in entries.items():
        t = np.array(ts_buf[eid], dtype=float)
        if sig.type in ("double", "int64", "float", "boolean"):
            v = np.array(val_buf[eid], dtype=float)
        else:
            v = val_buf[eid]
        out[sig.name] = LogSignal(name=sig.name, type=sig.type, t=t, v=v)
    return out


# --------------------------------------------------------------------------
# Mechanism / device discovery
# --------------------------------------------------------------------------

def find_sysid_state_signal(log: dict[str, LogSignal]):
    """Find the custom string signal carrying WPILib SysIdRoutine states."""
    candidates = []
    for name, sig in log.items():
        if sig.type != "string" or len(sig.v) == 0:
            continue
        uniq = set(sig.v)
        if not uniq or not uniq.issubset(SYSID_STATE_VALUES):
            continue
        n_active = sum(1 for x in sig.v if x != "none")
        if n_active == 0:
            continue
        candidates.append((name, sig, n_active))
    if not candidates:
        return None
    candidates.sort(key=lambda c: -c[2])
    name, sig, _ = candidates[0]
    if len(candidates) > 1:
        others = ", ".join(c[0] for c in candidates[1:])
        print(f"Note: other SysId state signals also present but unused: {others}",
              file=sys.stderr)
    return name, sig


def compute_mechanism_label(state_signal_name: str) -> tuple[str, str]:
    """Maps a SysIdRoutine state-signal name to a (friendly label, motor-group hint) pair.

    Returns:
        label: Human-readable description of the mechanism under test.
        group: Which motor group the fit should use: "drive", "steer", or "generic" when
            state_signal_name doesn't match one of CTRE's known swerve-generator names.
    """
    key = re.sub(r"[^a-z]", "", state_signal_name.lower())
    for prefix, (label, group) in KNOWN_SWERVE_TESTS.items():
        if key.startswith(prefix):
            return label, group
    stripped = re.sub(r"[_-]?state$", "", state_signal_name, flags=re.IGNORECASE)
    return f"Custom mechanism '{stripped}'", "generic"


def discover_talonfx_ids(log: dict[str, LogSignal]) -> list[int]:
    """Returns the sorted set of TalonFX CAN IDs with any signal present in log."""
    ids = set()
    for name in log:
        m = DEVICE_SIGNAL_RE.match(name)
        if m and m.group(1) == "TalonFX":
            ids.add(int(m.group(2)))
    return sorted(ids)


def get_talonfx_signal(log: dict[str, LogSignal], can_id: int, signal: str) -> LogSignal | None:
    """Returns the named status signal (e.g. "Velocity") for one TalonFX, or None if absent."""
    return log.get(f"Phoenix6/TalonFX-{can_id}/{signal}")


def cluster_motor_groups(log: dict[str, LogSignal], ids: list[int],
                          segments: list[Segment]) -> tuple[list[int], list[int]]:
    """Split TalonFX ids into (high-variance, low-variance) groups by commanded-voltage
    std during the test phases (segments) -- not the whole log, and not velocity.

    The motors the SysIdRoutine is actually driving are the ones it's commanding a
    swinging voltage to; the other group sits at ~0 V the whole time. This is more
    reliable than clustering on velocity: a motor that's being passively backdriven
    (e.g. a drive wheel scrubbing on carpet as its module's steer axis turns) can show
    more velocity variance than the motor that's actually under active control, which
    would make a velocity-based split pick the wrong group.
    """
    stds = {}
    for can_id in ids:
        sig = get_talonfx_signal(log, can_id, "MotorVoltage")
        if sig is None or len(sig.v) == 0:
            stds[can_id] = 0.0
            continue
        if segments:
            mask = np.zeros(len(sig.t), dtype=bool)
            for seg in segments:
                mask |= (sig.t >= seg.t_start) & (sig.t <= seg.t_end)
            vals = sig.v[mask]
        else:
            vals = sig.v
        stds[can_id] = float(np.std(vals)) if len(vals) else 0.0

    ordered = sorted(ids, key=lambda i: stds[i])
    if len(ordered) < 2:
        return ordered, []

    gaps = [stds[ordered[i + 1]] - stds[ordered[i]] for i in range(len(ordered) - 1)]
    split_index = np.argmax(gaps)
    split = int(split_index) + 1
    low = ordered[:split]
    high = ordered[split:]
    return high, low


# --------------------------------------------------------------------------
# TunerConstants.java best-effort gear-ratio/wheel-radius autodetection
# --------------------------------------------------------------------------

_UNIT_TO_METERS = {
    "Inches": 0.0254,
    "Inch": 0.0254,
    "Meters": 1.0,
    "Meter": 1.0,
    "Feet": 0.3048,
    "Centimeters": 0.01,
}

# Current CTRE Tuner X codegen names these DRIVE_GEAR_RATIO/STEER_GEAR_RATIO/WHEEL_RADIUS
# (e.g. this repo's CompbotTunerConstants.java); older generators used a k-prefixed name.
_GEAR_RATIO_FIELD_NAMES = {
    "drive": ("DRIVE_GEAR_RATIO", "kDriveGearRatio"),
    "steer": ("STEER_GEAR_RATIO", "kSteerGearRatio"),
}
_WHEEL_RADIUS_FIELD_NAMES = ("WHEEL_RADIUS", "kWheelRadius")


def autodetect_gearing(project_dir: str, motor_group: str) -> tuple[float, float | None, str] | None:
    """Best-effort search for a CTRE Tuner X generated TunerConstants*.java under project_dir.

    Returns:
        gear_ratio: Motor rotations per output rotation, read from the drive/steer gear ratio field.
        wheel_radius_m: Drive wheel radius, m, read from the wheel radius field; None for a steer motor_group.
        source_path: Path to the TunerConstants file the values came from.
        Or None if no TunerConstants file, or no matching gear-ratio field, was found.
    """
    pattern = os.path.join(project_dir, "**", "*TunerConstants*.java")
    matches = glob.glob(pattern, recursive=True)
    matches = [m for m in matches if "/build/" not in m and "\\build\\" not in m]
    if not matches:
        return None
    path = matches[0]
    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            text = f.read()
    except OSError:
        return None

    gear_key_pattern = "|".join(_GEAR_RATIO_FIELD_NAMES[motor_group])
    m_gear = re.search(rf"(?:{gear_key_pattern})\s*=\s*([0-9.]+)", text)
    if not m_gear:
        return None
    gear_ratio = float(m_gear.group(1))

    wheel_radius_m = None
    if motor_group == "drive":
        wheel_key_pattern = "|".join(_WHEEL_RADIUS_FIELD_NAMES)
        m_wheel = re.search(rf"(?:{wheel_key_pattern})\s*=\s*(\w+)\.of\(([0-9.]+)\)", text)
        if m_wheel:
            unit, value = m_wheel.group(1), float(m_wheel.group(2))
            scale = _UNIT_TO_METERS.get(unit)
            if scale is not None:
                wheel_radius_m = value * scale

    return gear_ratio, wheel_radius_m, path


# --------------------------------------------------------------------------
# Fitting
# --------------------------------------------------------------------------

@dataclass
class Segment:
    phase: str
    t_start: float
    t_end: float


def build_segments(state_sig: LogSignal) -> list[Segment]:
    """Splits a SysIdRoutine state signal into contiguous, non-"none" test-phase segments.

    t_start and t_end are the timestamps of the segment's own first and last sample
    (inclusive), never a neighboring segment's -- a phase change doesn't donate its first
    sample to the segment before it.
    """
    segments: list[Segment] = []
    if len(state_sig.t) == 0:
        return segments
    cur_phase = state_sig.v[0]
    cur_start = state_sig.t[0]
    for i in range(1, len(state_sig.t)):
        if state_sig.v[i] != cur_phase:
            segments.append(Segment(cur_phase, cur_start, state_sig.t[i - 1]))
            cur_phase = state_sig.v[i]
            cur_start = state_sig.t[i]
    segments.append(Segment(cur_phase, cur_start, state_sig.t[-1]))
    return [s for s in segments if s.phase != "none" and s.t_end > s.t_start]


_EMPTY_ROWS = (np.array([]), np.array([]), np.array([]), np.array([]))


def _concat_rows(rows: dict[str, list]):
    """Concatenates a {"sign": [...], "v": [...], "a": [...], "volt": [...]} row bucket."""
    if not rows["v"]:
        return _EMPTY_ROWS
    return (np.concatenate(rows["sign"]), np.concatenate(rows["v"]),
            np.concatenate(rows["a"]), np.concatenate(rows["volt"]))


def extract_motor_rows(log: dict[str, LogSignal], can_id: int, segments: list[Segment],
                        trim_s: float, quasi_vel_threshold: float):
    """Builds this motor's fit rows (sign(v), v, a, voltage), split by phase type.

    Within each segment (after trim_s), velocity is resampled onto a uniform time grid --
    spaced at that segment's own median sample interval, so this neither up- nor down-samples
    the data, just regularizes jitter in the raw CAN bus timestamps -- before taking the
    gradient for acceleration. A non-uniform finite difference (the previous approach) biases
    the acceleration estimate when neighboring samples aren't evenly spaced; this also means a
    fixed sample-count smoothing window, if one were added later, would correspond to a
    consistent span of time, unlike a window over the raw samples.

    Args:
        trim_s: Seconds dropped from the start of each segment, to skip state-transition transients.
        quasi_vel_threshold: Rotor rot/s; quasistatic samples (after resampling) below this
            |velocity| are dropped so pre-breakaway static-friction noise doesn't bias kV.

    Returns:
        (quasi, dynamic): Each a (sign_v, v, a, volt) tuple of one row per kept sample, pooled
            across that motor's quasistatic-* or dynamic-* segments respectively; empty arrays
            in either slot if can_id has no usable data of that phase type.
    """
    vel_sig = get_talonfx_signal(log, can_id, "Velocity")
    volt_sig = get_talonfx_signal(log, can_id, "MotorVoltage")
    if vel_sig is None or volt_sig is None or len(vel_sig.t) < 3:
        return _EMPTY_ROWS, _EMPTY_ROWS

    t = vel_sig.t
    v = vel_sig.v

    quasi_rows = {"sign": [], "v": [], "a": [], "volt": []}
    dyn_rows = {"sign": [], "v": [], "a": [], "volt": []}

    for seg in segments:
        lo = np.searchsorted(t, seg.t_start + trim_s, side="left")
        hi = np.searchsorted(t, seg.t_end, side="right")
        if hi - lo < 3:
            continue
        t_seg = t[lo:hi]
        v_seg = v[lo:hi]

        duration = t_seg[-1] - t_seg[0]
        if duration <= 0:
            continue
        dt = np.median(np.diff(t_seg))
        if dt <= 0:
            dt = duration / (len(t_seg) - 1)
        n = max(3, int(round(duration / dt)) + 1)
        t_uniform = np.linspace(t_seg[0], t_seg[-1], n)
        dt_actual = t_uniform[1] - t_uniform[0]

        v_uniform = np.interp(t_uniform, t_seg, v_seg)
        volt_uniform = np.interp(t_uniform, volt_sig.t, volt_sig.v)
        a_uniform = np.gradient(v_uniform, dt_actual)

        mask = np.ones_like(v_uniform, dtype=bool)
        bucket = dyn_rows
        if seg.phase.startswith("quasistatic"):
            mask &= np.abs(v_uniform) >= quasi_vel_threshold
            bucket = quasi_rows

        bucket["sign"].append(np.sign(v_uniform[mask]))
        bucket["v"].append(v_uniform[mask])
        bucket["a"].append(a_uniform[mask])
        bucket["volt"].append(volt_uniform[mask])

    return _concat_rows(quasi_rows), _concat_rows(dyn_rows)


def _r_squared(actual: np.ndarray, pred: np.ndarray) -> float:
    """Coefficient of determination; NaN if actual is constant."""
    ss_res = float(np.sum((actual - pred) ** 2))
    ss_tot = float(np.sum((actual - np.mean(actual)) ** 2))
    return 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")


def fit_ks_kv(sign_v: np.ndarray, v: np.ndarray, volt: np.ndarray):
    """Fits V = kS*sign_v + kV*v by ordinary least squares, over quasistatic data.

    Acceleration is left out of this stage: quasistatic velocity is ramped slowly enough
    that acceleration is assumed negligible, so including it here would mean fitting kA to
    mostly noise.

    Args:
        sign_v: Sign of velocity, one row per sample.
        v: Velocity, rotor rot/s, one row per sample.
        volt: Motor voltage, V, one row per sample; the regression target.

    Returns:
        ks, kv: Fitted feedforward gains.
        r2: Coefficient of determination of the fit; NaN if volt is constant.
        n: Number of samples fit over.
    """
    A = np.column_stack([sign_v, v])
    coeffs, _, _, _ = np.linalg.lstsq(A, volt, rcond=None)
    ks, kv = coeffs
    r2 = _r_squared(volt, A @ coeffs)
    return ks, kv, r2, len(volt)


def fit_ka(a: np.ndarray, residual_volt: np.ndarray):
    """Fits residual_volt = kA*a by ordinary least squares (forced through the origin), over
    dynamic data, with kS/kV already subtracted out of residual_volt by the caller.

    Args:
        a: Acceleration, rotor rot/s^2, one row per sample.
        residual_volt: Motor voltage minus the quasistatic kS*sign(v) + kV*v prediction, one
            row per sample; the regression target.

    Returns:
        ka: Fitted feedforward gain.
        r2: Coefficient of determination of the fit; NaN if residual_volt is constant.
        n: Number of samples fit over.
    """
    denom = float(np.dot(a, a))
    ka = float(np.dot(a, residual_volt) / denom) if denom > 0 else float("nan")
    r2 = _r_squared(residual_volt, ka * a)
    return ka, r2, len(residual_volt)


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log_file", help="Path to a raw Phoenix6 .hoot log, or an already-converted "
                                      ".wpilog (e.g. one opened in AdvantageScope)")
    ap.add_argument("--owlet", default=None,
                     help="Path to the owlet binary, needed for a .hoot log (default: first match on PATH)")
    ap.add_argument("--project-dir", default=".",
                     help="Directory to search for TunerConstants*.java for unit conversion (default: cwd)")
    ap.add_argument("--gear-ratio", type=float, default=None,
                     help="Override auto-detected gear ratio (motor rotations per output rotation)")
    ap.add_argument("--wheel-radius-m", type=float, default=None,
                     help="Override auto-detected drive wheel radius in meters")
    ap.add_argument("--trim-s", type=float, default=0.1,
                     help="Seconds to trim from the start of each test phase (default 0.1)")
    ap.add_argument("--quasi-vel-threshold", type=float, default=0.1,
                     help="Min |velocity| (rotor rot/s) to keep in quasistatic phases (default 0.1)")
    ap.add_argument("--keep-wpilog", default=None,
                     help="Save the .hoot log's converted .wpilog to this path (ignored for a .wpilog input)")
    ap.add_argument("--plot", default=None,
                     help="Save a diagnostic plot to this path, or pass 'imshow' to pop up "
                          "an interactive matplotlib window instead of saving a file")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    if args.log_file.lower().endswith(".wpilog"):
        log = read_wpilog(args.log_file)
    else:
        try:
            owlet = find_owlet(args.owlet)
        except (FileNotFoundError, RuntimeError) as e:
            print(f"Error: {e}", file=sys.stderr)
            sys.exit(1)

        if args.keep_wpilog:
            wpilog_path = args.keep_wpilog
            convert_hoot_to_wpilog(owlet, args.log_file, wpilog_path, quiet=args.quiet)
            log = read_wpilog(wpilog_path)
        else:
            with tempfile.TemporaryDirectory() as tmp:
                wpilog_path = os.path.join(tmp, "converted.wpilog")
                convert_hoot_to_wpilog(owlet, args.log_file, wpilog_path, quiet=args.quiet)
                log = read_wpilog(wpilog_path)

    found = find_sysid_state_signal(log)
    if found is None:
        print("No WPILib SysIdRoutine state signal found in this log -- "
              "it doesn't look like a SysId characterization run.", file=sys.stderr)
        sys.exit(1)
    state_name, state_sig = found
    label, group_hint = compute_mechanism_label(state_name)

    print(f"Detected mechanism: {label}")
    print(f"  (from custom signal '{state_name}')")

    segments = build_segments(state_sig)

    talon_ids = discover_talonfx_ids(log)
    if not talon_ids:
        print("No TalonFX devices found in the log.", file=sys.stderr)
        sys.exit(1)

    if group_hint == "generic" or len(talon_ids) < 2:
        motor_ids = talon_ids
        other_ids: list[int] = []
        print(f"TalonFX devices on bus: {talon_ids} (using all of them)")
    else:
        # the motors actually driven during the test are always the ones with
        # swinging commanded voltage, regardless of whether this was a drive or steer test
        motor_ids, other_ids = cluster_motor_groups(log, talon_ids, segments)

        active_name = "steer motors"
        quiescent_name = "drive wheels"
        if group_hint == "drive":
            active_name = "drive wheels"
            quiescent_name = "steer motors"

        print(f"TalonFX devices on bus: {talon_ids}")
        print(f"  Active during test (CAN IDs): {motor_ids}  <- these are the {active_name}")
        print(f"  Quiescent during test (CAN IDs): {other_ids}  <- {quiescent_name}")

    phase_counts = {}
    for seg in segments:
        phase_counts[seg.phase] = phase_counts.get(seg.phase, 0) + 1
    print(f"Test phases found: {phase_counts}")

    quasi_sign_l, quasi_v_l, quasi_a_l, quasi_volt_l = [], [], [], []
    dyn_sign_l, dyn_v_l, dyn_a_l, dyn_volt_l = [], [], [], []
    for can_id in motor_ids:
        (qs, qv, qa, qvolt), (ds, dv, da, dvolt) = extract_motor_rows(
            log, can_id, segments, args.trim_s, args.quasi_vel_threshold)
        if len(qv) == 0 and len(dv) == 0:
            print(f"  warning: no usable samples for TalonFX-{can_id}", file=sys.stderr)
            continue
        quasi_sign_l.append(qs); quasi_v_l.append(qv); quasi_a_l.append(qa); quasi_volt_l.append(qvolt)
        dyn_sign_l.append(ds); dyn_v_l.append(dv); dyn_a_l.append(da); dyn_volt_l.append(dvolt)

    quasi_sign = np.concatenate(quasi_sign_l) if quasi_sign_l else np.array([])
    quasi_v = np.concatenate(quasi_v_l) if quasi_v_l else np.array([])
    quasi_volt = np.concatenate(quasi_volt_l) if quasi_volt_l else np.array([])
    dyn_sign = np.concatenate(dyn_sign_l) if dyn_sign_l else np.array([])
    dyn_v = np.concatenate(dyn_v_l) if dyn_v_l else np.array([])
    dyn_a = np.concatenate(dyn_a_l) if dyn_a_l else np.array([])
    dyn_volt = np.concatenate(dyn_volt_l) if dyn_volt_l else np.array([])

    if len(quasi_v) == 0:
        print("No usable quasistatic-phase data extracted; cannot fit kS/kV.", file=sys.stderr)
        sys.exit(1)
    if len(dyn_v) == 0:
        print("No usable dynamic-phase data extracted; cannot fit kA.", file=sys.stderr)
        sys.exit(1)

    ks, kv, r2_quasi, n_quasi = fit_ks_kv(quasi_sign, quasi_v, quasi_volt)
    dyn_residual = dyn_volt - ks * dyn_sign - kv * dyn_v
    ka, r2_dyn, n_dyn = fit_ka(dyn_a, dyn_residual)

    print()
    print(f"Stage A, quasistatic phases ({n_quasi} samples, {len(motor_ids)} motor(s)): "
          f"kS/kV fit, R^2 = {r2_quasi:.4f}")
    print(f"  kS = {ks:.4f} V")
    print(f"  kV = {kv:.5f} V/(rotor rot/s)")
    print()
    print(f"Stage B, dynamic phases ({n_dyn} samples, {len(motor_ids)} motor(s)): "
          f"kA fit (kS/kV held fixed from Stage A), R^2 = {r2_dyn:.4f}")
    print(f"  kA = {ka:.5f} V/(rotor rot/s^2)")

    gear_ratio = args.gear_ratio
    wheel_radius_m = args.wheel_radius_m
    source = "command line"
    if gear_ratio is None and group_hint in ("drive", "steer"):
        detected = autodetect_gearing(args.project_dir, group_hint)
        if detected:
            gear_ratio, wheel_radius_m_detected, source = detected
            if wheel_radius_m is None:
                wheel_radius_m = wheel_radius_m_detected
            print(f"\nAuto-detected gearing from {source}:")
            print(f"  gear ratio = {gear_ratio} motor rot / output rot")
            if wheel_radius_m:
                print(f"  wheel radius = {wheel_radius_m * 1000:.2f} mm")

    if gear_ratio:
        kv_out = kv * gear_ratio
        ka_out = ka * gear_ratio
        print(f"\nConverted to output-shaft units (gear ratio {gear_ratio}):")
        print(f"  kV = {kv_out:.5f} V/(output rot/s)")
        print(f"  kA = {ka_out:.5f} V/(output rot/s^2)")
        if wheel_radius_m and group_hint == "drive":
            circumference = 2 * np.pi * wheel_radius_m
            kv_lin = kv_out / circumference
            ka_lin = ka_out / circumference
            print(f"  kV = {kv_lin:.5f} V/(m/s)")
            print(f"  kA = {ka_lin:.5f} V/(m/s^2)")
    else:
        print("\n(No gear ratio available -- pass --gear-ratio / --wheel-radius-m, "
              "or run with --project-dir pointing at a Tuner X project, for physical units.)")

    if args.plot:
        plot_args = (quasi_v, quasi_volt, dyn_a, dyn_residual, ks, kv, ka, label)
        if args.plot.lower() == "imshow":
            show_plot(*plot_args)
        else:
            make_plot(args.plot, *plot_args)
            print(f"\nSaved diagnostic plot to {args.plot}")


def _build_plot_figure(quasi_v, quasi_volt, dyn_a, dyn_residual, ks, kv, ka, label):
    """Builds the two-panel diagnostic figure: Stage A's quasistatic voltage-vs-velocity fit,
    and Stage B's dynamic residual-voltage-vs-acceleration fit."""
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))

    axes[0].scatter(quasi_v, quasi_volt, s=4, alpha=0.4, label="quasistatic data")
    vs = np.linspace(quasi_v.min(), quasi_v.max(), 200)
    axes[0].plot(vs, ks * np.sign(vs) + kv * vs, color="orange", label="kS*sign(v) + kV*v")
    axes[0].set_xlabel("velocity (rotor rot/s)")
    axes[0].set_ylabel("voltage (V)")
    axes[0].set_title("Stage A: voltage vs. velocity (quasistatic)")
    axes[0].legend()

    axes[1].scatter(dyn_a, dyn_residual, s=4, alpha=0.4, label="dynamic data")
    as_ = np.linspace(dyn_a.min(), dyn_a.max(), 200)
    axes[1].plot(as_, ka * as_, color="orange", label="kA*a")
    axes[1].set_xlabel("acceleration (rotor rot/s^2)")
    axes[1].set_ylabel("voltage - kS*sign(v) - kV*v (V)")
    axes[1].set_title("Stage B: residual voltage vs. acceleration (dynamic)")
    axes[1].legend()

    fig.suptitle(label)
    fig.tight_layout()
    return fig


def make_plot(path, quasi_v, quasi_volt, dyn_a, dyn_residual, ks, kv, ka, label):
    """Saves the diagnostic figure to path."""
    import matplotlib
    matplotlib.use("Agg")
    fig = _build_plot_figure(quasi_v, quasi_volt, dyn_a, dyn_residual, ks, kv, ka, label)
    fig.savefig(path, dpi=150)


def show_plot(quasi_v, quasi_volt, dyn_a, dyn_residual, ks, kv, ka, label):
    """Pops up the diagnostic figure in an interactive matplotlib window."""
    import matplotlib.pyplot as plt
    _build_plot_figure(quasi_v, quasi_volt, dyn_a, dyn_residual, ks, kv, ka, label)
    plt.show()


if __name__ == "__main__":
    main()
