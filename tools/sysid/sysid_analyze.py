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
  4. Fits the standard SysId feedforward model
         V = kS*sign(v) + kV*v + kA*a
     by ordinary least squares over the quasistatic + dynamic test phases,
     pooling data from all motors in the identified group, and reports
     kS/kV/kA plus R^2, matching what the WPILib SysId "Simple/Drivetrain"
     feedforward analysis produces.

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


def cluster_motor_groups(log: dict[str, LogSignal], ids: list[int]) -> tuple[list[int], list[int]]:
    """Split TalonFX ids into (high-variance, low-variance) groups by velocity std.

    For a swerve drivetrain this cleanly separates the 4 motors that are
    actually being driven during the test from the 4 that are being held
    still, with no CAN-ID assumptions.
    """
    stds = {}
    for can_id in ids:
        sig = get_talonfx_signal(log, can_id, "Velocity")
        if sig is None or len(sig.v) == 0:
            stds[can_id] = 0.0
            continue
        velocity_std = np.std(sig.v)
        stds[can_id] = float(velocity_std)

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


def extract_motor_rows(log: dict[str, LogSignal], can_id: int, segments: list[Segment],
                        trim_s: float, quasi_vel_threshold: float):
    """Builds this motor's fit rows (sign(v), v, a, voltage) over the given test segments.

    Args:
        trim_s: Seconds dropped from the start of each segment, to skip state-transition transients.
        quasi_vel_threshold: Rotor rot/s; quasistatic samples below this |velocity| are dropped so
            pre-breakaway static-friction noise doesn't bias kV.

    Returns:
        sign_v, v, a, volt: One row per kept sample; empty arrays if can_id has no usable data.
    """
    vel_sig = get_talonfx_signal(log, can_id, "Velocity")
    volt_sig = get_talonfx_signal(log, can_id, "MotorVoltage")
    if vel_sig is None or volt_sig is None or len(vel_sig.t) < 3:
        return np.array([]), np.array([]), np.array([]), np.array([])

    t = vel_sig.t
    v = vel_sig.v
    volt = np.interp(t, volt_sig.t, volt_sig.v)

    sign_rows, v_rows, a_rows, volt_rows = [], [], [], []
    for seg in segments:
        lo = np.searchsorted(t, seg.t_start + trim_s, side="left")
        hi = np.searchsorted(t, seg.t_end, side="right")
        if hi - lo < 3:
            continue
        t_seg = t[lo:hi]
        v_seg = v[lo:hi]
        volt_seg = volt[lo:hi]
        a_seg = np.gradient(v_seg, t_seg)

        mask = np.ones_like(v_seg, dtype=bool)
        if seg.phase.startswith("quasistatic"):
            mask &= np.abs(v_seg) >= quasi_vel_threshold

        sign_rows.append(np.sign(v_seg[mask]))
        v_rows.append(v_seg[mask])
        a_rows.append(a_seg[mask])
        volt_rows.append(volt_seg[mask])

    if not v_rows:
        return np.array([]), np.array([]), np.array([]), np.array([])
    return (np.concatenate(sign_rows), np.concatenate(v_rows),
            np.concatenate(a_rows), np.concatenate(volt_rows))


def fit_feedforward(sign_v: np.ndarray, v: np.ndarray, a: np.ndarray, volt: np.ndarray):
    """Fits V = kS*sign_v + kV*v + kA*a by ordinary least squares.

    Args:
        sign_v: Sign of velocity, one row per sample.
        v: Velocity, rotor rot/s, one row per sample.
        a: Acceleration, rotor rot/s^2, one row per sample.
        volt: Motor voltage, V, one row per sample; the regression target.

    Returns:
        ks, kv, ka: Fitted feedforward gains.
        r2: Coefficient of determination of the fit; NaN if volt is constant.
        n: Number of samples fit over.
    """
    A = np.column_stack([sign_v, v, a])
    coeffs, residuals, rank, sv = np.linalg.lstsq(A, volt, rcond=None)
    ks, kv, ka = coeffs
    pred = A @ coeffs

    sq_residual_sum = np.sum((volt - pred) ** 2)
    ss_res = float(sq_residual_sum)
    volt_mean = np.mean(volt)
    sq_deviation_sum = np.sum((volt - volt_mean) ** 2)
    ss_tot = float(sq_deviation_sum)

    r2 = float("nan")
    if ss_tot > 0:
        r2 = 1.0 - ss_res / ss_tot
    return ks, kv, ka, r2, len(volt)


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

    talon_ids = discover_talonfx_ids(log)
    if not talon_ids:
        print("No TalonFX devices found in the log.", file=sys.stderr)
        sys.exit(1)

    if group_hint == "generic" or len(talon_ids) < 2:
        motor_ids = talon_ids
        other_ids: list[int] = []
        print(f"TalonFX devices on bus: {talon_ids} (using all of them)")
    else:
        # the motors actually driven during the test are always the high-variance group,
        # regardless of whether this was a drive or a steer test
        motor_ids, other_ids = cluster_motor_groups(log, talon_ids)

        active_name = "steer motors"
        quiescent_name = "drive wheels"
        if group_hint == "drive":
            active_name = "drive wheels"
            quiescent_name = "steer motors"

        print(f"TalonFX devices on bus: {talon_ids}")
        print(f"  Active during test (CAN IDs): {motor_ids}  <- these are the {active_name}")
        print(f"  Quiescent during test (CAN IDs): {other_ids}  <- {quiescent_name}")

    segments = build_segments(state_sig)
    phase_counts = {}
    for seg in segments:
        phase_counts[seg.phase] = phase_counts.get(seg.phase, 0) + 1
    print(f"Test phases found: {phase_counts}")

    all_sign, all_v, all_a, all_volt = [], [], [], []
    for can_id in motor_ids:
        s, v, a, volt = extract_motor_rows(log, can_id, segments, args.trim_s,
                                            args.quasi_vel_threshold)
        if len(v) == 0:
            print(f"  warning: no usable samples for TalonFX-{can_id}", file=sys.stderr)
            continue
        all_sign.append(s)
        all_v.append(v)
        all_a.append(a)
        all_volt.append(volt)

    if not all_v:
        print("No usable data extracted; cannot fit.", file=sys.stderr)
        sys.exit(1)

    sign_v = np.concatenate(all_sign)
    v = np.concatenate(all_v)
    a = np.concatenate(all_a)
    volt = np.concatenate(all_volt)

    ks, kv, ka, r2, n = fit_feedforward(sign_v, v, a, volt)

    print()
    print(f"Fit over {n} samples pooled from {len(motor_ids)} motor(s), R^2 = {r2:.4f}")
    print(f"  kS = {ks:.4f} V")
    print(f"  kV = {kv:.5f} V/(rotor rot/s)")
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
        if args.plot.lower() == "imshow":
            show_plot(sign_v, v, a, volt, ks, kv, ka, label)
        else:
            make_plot(args.plot, sign_v, v, a, volt, ks, kv, ka, label)
            print(f"\nSaved diagnostic plot to {args.plot}")


def _build_plot_figure(sign_v, v, a, volt, ks, kv, ka, label):
    """Builds the two-panel voltage-vs-velocity / residual-vs-acceleration diagnostic figure."""
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))

    axes[0].scatter(v, volt, s=4, alpha=0.4, label="data")
    vs = np.linspace(v.min(), v.max(), 200)
    axes[0].plot(vs, ks * np.sign(vs) + kv * vs, color="orange", label="kS*sign(v) + kV*v")
    axes[0].set_xlabel("velocity (rotor rot/s)")
    axes[0].set_ylabel("voltage (V)")
    axes[0].set_title("Voltage vs. velocity")
    axes[0].legend()

    residual_v = volt - ks * sign_v - kv * v
    axes[1].scatter(a, residual_v, s=4, alpha=0.4, label="data")
    as_ = np.linspace(a.min(), a.max(), 200)
    axes[1].plot(as_, ka * as_, color="orange", label="kA*a")
    axes[1].set_xlabel("acceleration (rotor rot/s^2)")
    axes[1].set_ylabel("voltage - kS*sign(v) - kV*v (V)")
    axes[1].set_title("Residual voltage vs. acceleration")
    axes[1].legend()

    fig.suptitle(label)
    fig.tight_layout()
    return fig


def make_plot(path, sign_v, v, a, volt, ks, kv, ka, label):
    """Saves the diagnostic figure to path."""
    import matplotlib
    matplotlib.use("Agg")
    fig = _build_plot_figure(sign_v, v, a, volt, ks, kv, ka, label)
    fig.savefig(path, dpi=150)


def show_plot(sign_v, v, a, volt, ks, kv, ka, label):
    """Pops up the diagnostic figure in an interactive matplotlib window."""
    import matplotlib.pyplot as plt
    _build_plot_figure(sign_v, v, a, volt, ks, kv, ka, label)
    plt.show()


if __name__ == "__main__":
    main()
