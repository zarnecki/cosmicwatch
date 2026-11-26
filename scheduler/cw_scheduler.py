#!/usr/bin/env python3
# coding: utf-8
"""
cw_scheduler.py - run an observation plan for the CosmicWatch rotating mount.

Usage:  python3 cw_scheduler.py <plan_file>

Plan file: one observation per line, three fields separated by spaces/tabs:

    <angle>  <time>  <output_file>

  angle        mount angle in degrees, -90 .. 90 (e.g. 0, -45, 30.5)
  time         observation time with unit: h, m or s; units can be combined
               (e.g. 30s, 15m, 2h, 1.5h, 1h30m); allowed range 10 s .. 24 h
  output_file  data file name, written in the same directory as the plan;
               letters, digits, '_', '-', '.', '+' only (must start with
               a letter or digit); must not exist yet and must be unique in the plan

Empty lines and everything after '#' are ignored. Example:

    # angle  time   file
      0      2h     run_000deg.txt
      45     90m    run_p45deg.txt
     -45     1h30m  run_m45deg.txt

For every line the scheduler
  1. moves the mount:   python3 motor_control.py <angle>
  2. collects data:     cw_reader <plan_dir>/<output_file> <seconds>
                        (started in the directory that contains dev/CW0..7)
  3. checks the mount:  python3 motor_control.py read
                        (position after data taking; a change of more than
                        POSITION_TOLERANCE is reported as a warning and a note)
The plan is checked completely before anything is moved. After the check
the expected total time is shown and the plan starts only after confirmation.
Everything shown on the screen is also written to a log file next to the
plan: <plan_name>_<YYYYmmdd_HHMMSS>.log

Ctrl-C stops the current step cleanly (motor switched off / data file
closed) and ends the plan with a summary.

Exit code: 0 = all observations done, 1 = plan stopped (error or Ctrl-C),
2 = plan file or setup not correct (nothing was done).
"""

import math
import os
import re
import subprocess
import sys
import time

#------------------------------------------------------------------------
# Setup - adjust to the installation on the Raspberry Pi.
# Each value can also be overridden with an environment variable of the
# same name (e.g. CW_READER=/home/pi/CW/cw_reader python3 cw_scheduler.py plan.txt).
# Relative paths are taken relative to the directory of this script.

SETUP = {
    "MOTOR_CONTROL": "motor_control.py",   # motor positioning script
    "CW_READER":     "cw_reader",          # compiled readout program
    "CW_READER_DIR": ".",                  # directory containing dev/CW0..dev/CW7
    "PYTHON":        sys.executable or "python3",   # interpreter for motor_control.py
}

# Plan limits
ANGLE_MIN, ANGLE_MAX = -90.0, 90.0       # deg
TIME_MIN, TIME_MAX = 10.0, 24 * 3600.0   # s
MAX_NAME_LEN = 100
NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.+\-]*$")
TIME_RE = re.compile(r"^(?:(\d+(?:\.\d*)?|\.\d+)h)?(?:(\d+(?:\.\d*)?|\.\d+)m)?"
                     r"(?:(\d+(?:\.\d*)?|\.\d+)s)?$", re.IGNORECASE)
ANGLE_RE = re.compile(r"^[+-]?(\d+(\.\d*)?|\.\d+)$")

# Positioning and handling of problems during the run
POSITION_TOLERANCE = 0.5  # deg; final position (after motor off) farther from the target
                          # -> correction (motor_control.py positions to ANGLE_TOLERANCE = 0.2
                          # with the motor on; this limit is for the position after motor off).
                          # Also the limit for a change of the position during data taking.
MAX_CORRECTIONS = 3       # corrections after the first move; each one approaches the
                          # target from the other side: first OVERSHOOT deg past the target
                          # (in the direction of the first move), then back to the target
OVERSHOOT = 5.0           # deg
MOTOR_LIMIT = 96.0        # deg, range accepted by motor_control.py (ANGLE_MIN/MAX there);
                          # the overshoot is clipped to it (plans themselves stay within +-90)
POSITION_LIMIT = 5.0      # deg; if the position after all corrections is still farther
                          # from the target, the plan is stopped (otherwise data are
                          # taken at the reached angle, which is logged)
MIN_TIME_FRACTION = 0.95  # readout shorter than this fraction of the requested
                          # time = detectors lost -> plan is stopped

# Time estimate (from motor_control.py parameters and the 12 Nov lab test)
SEC_PER_DEG = 2 * 0.025 / 0.288     # 20 steps/s, 0.288 deg/step -> 0.174 s/deg
MOVE_OVERHEAD = 8.0                 # s: start-up, checking reads, motor-off reads
READOUT_OVERHEAD = 7.0              # s: opening ports + waiting for all detector headers
                                    # (cw_reader >= 20261117 counts the time from then; 5-6 s seen)
START_ANGLE_GUESS = 0.0             # deg, assumed position before the first move

#------------------------------------------------------------------------
# Output: screen + log file

class Output:
    def __init__(self):
        self.log = None
    def open_log(self, path):
        self.log = open(path, "w", buffering=1)
    def __call__(self, msg="", stamp=False):
        if stamp:
            msg = f"{time.strftime('%H:%M:%S')} | {msg}"
        print(msg, flush=True)
        if self.log:
            self.log.write(msg + "\n")
    def close(self):
        if self.log:
            self.log.close()
            self.log = None

out = Output()

def fmt_time(sec):
    """Seconds -> 'XhYYmZZs' style string."""
    sec = int(round(sec))
    h, rest = divmod(sec, 3600)
    m, s = divmod(rest, 60)
    if h:
        return f"{h}h{m:02d}m{s:02d}s"
    if m:
        return f"{m}m{s:02d}s"
    return f"{s}s"

def fmt_seconds_arg(sec):
    """Seconds as argument for cw_reader (no exponent notation)."""
    return str(int(sec)) if float(sec).is_integer() else f"{sec:.3f}".rstrip("0")

#------------------------------------------------------------------------
# Plan checking

def parse_time(text):
    """'1h30m' -> 5400.0; None if the format is wrong."""
    m = TIME_RE.match(text)
    if not m or not any(m.groups()):
        return None
    h, mi, s = (float(g) if g else 0.0 for g in m.groups())
    return h * 3600 + mi * 60 + s

def check_plan(plan_path):
    """Returns (steps, errors). steps: list of dicts; errors: list of strings."""
    errors = []
    steps = []
    try:
        with open(plan_path, "r", encoding="utf-8") as f:
            lines = f.readlines()
    except UnicodeDecodeError:
        return [], ["plan file is not a text file (UTF-8/ASCII expected)"]
    except OSError as e:
        return [], [f"cannot read plan file: {e.strerror}"]

    plan_dir = os.path.dirname(os.path.abspath(plan_path))
    plan_name = os.path.basename(plan_path)
    seen = {}

    for n, line in enumerate(lines, start=1):
        text = line.split("#", 1)[0].strip()
        if not text:
            continue
        fields = text.split()
        if len(fields) != 3:
            hint = " - no spaces allowed in file names" if len(fields) > 3 else ""
            errors.append(f"line {n}: expected 3 fields (angle time file), found {len(fields)}{hint}: '{text}'")
            continue
        a_txt, t_txt, name = fields
        ok = True

        # angle
        angle = None
        if not ANGLE_RE.match(a_txt):
            errors.append(f"line {n}: angle '{a_txt}' is not a number "
                          f"(give degrees without unit, e.g. 30 or -45.5)")
            ok = False
        else:
            angle = float(a_txt)
            if not ANGLE_MIN <= angle <= ANGLE_MAX:
                errors.append(f"line {n}: angle {a_txt} outside allowed range "
                              f"{ANGLE_MIN:g} .. {ANGLE_MAX:g} deg")
                ok = False

        # time
        sec = parse_time(t_txt)
        if sec is None:
            errors.append(f"line {n}: time '{t_txt}' has wrong format "
                          f"(use a number with unit h, m or s, e.g. 45s, 20m, 1.5h, 1h30m)")
            ok = False
        elif not TIME_MIN <= sec <= TIME_MAX:
            errors.append(f"line {n}: time '{t_txt}' = {sec:g} s outside allowed range "
                          f"{TIME_MIN:g} s .. {TIME_MAX / 3600:g} h")
            ok = False

        # file name
        if "/" in name or "\\" in name:
            errors.append(f"line {n}: file name '{name}' must not contain a path "
                          f"(files are written to the plan directory)")
            ok = False
        elif not NAME_RE.match(name):
            bad = sorted(set(c for c in name if not re.match(r"[A-Za-z0-9_.+\-]", c)))
            if bad:
                errors.append(f"line {n}: file name '{name}' contains not allowed character(s) "
                              f"{' '.join(repr(c) for c in bad)} (allowed: letters, digits, _ - . +)")
            else:
                errors.append(f"line {n}: file name '{name}' must start with a letter or digit")
            ok = False
        elif len(name) > MAX_NAME_LEN:
            errors.append(f"line {n}: file name '{name}' too long (max {MAX_NAME_LEN} characters)")
            ok = False
        elif name == plan_name:
            errors.append(f"line {n}: file name '{name}' is the plan file itself")
            ok = False
        else:
            if name in seen:
                errors.append(f"line {n}: file name '{name}' already used in line {seen[name]}")
                ok = False
            else:
                seen[name] = n
            if os.path.lexists(os.path.join(plan_dir, name)):
                errors.append(f"line {n}: file '{name}' already exists in {plan_dir}")
                ok = False

        if ok:
            steps.append({"line": n, "angle": angle, "time_txt": t_txt, "seconds": sec,
                          "name": name, "path": os.path.join(plan_dir, name)})

    if not lines or (not steps and not errors):
        errors.append("plan contains no observations")
    if not os.access(plan_dir, os.W_OK):
        errors.append(f"no write permission in the plan directory {plan_dir}")
    return steps, errors

def resolve_setup():
    """Paths of the programs; returns (setup, errors)."""
    here = os.path.dirname(os.path.abspath(__file__))
    s, errors = {"warnings": []}, []
    for key, val in SETUP.items():
        val = os.environ.get(key, val)
        if key != "PYTHON" and not os.path.isabs(val):
            val = os.path.normpath(os.path.join(here, val))
        s[key] = val
    if not os.path.isfile(s["MOTOR_CONTROL"]):
        errors.append(f"motor control script not found: {s['MOTOR_CONTROL']}")
    if not (os.path.isfile(s["CW_READER"]) and os.access(s["CW_READER"], os.X_OK)):
        errors.append(f"readout program not found or not executable: {s['CW_READER']}")
    devdir = os.path.join(s["CW_READER_DIR"], "dev")
    if not os.path.isdir(devdir):
        errors.append(f"detector link directory not found: {devdir}")
    else:
        missing = [f"CW{i}" for i in range(8) if not os.path.exists(os.path.join(devdir, f"CW{i}"))]
        if len(missing) == 8:
            errors.append(f"no detector connected ({devdir}/CW0..CW7 do not exist)")
        elif missing:
            s["warnings"].append(f"detector(s) not connected: {', '.join(missing)}")
    if os.path.isfile(s["CW_READER"]):
        msg = check_reader_caps(s["CW_READER"])
        if msg:
            s["warnings"].append(msg)
    return s, errors

# Linux capabilities used by cw_reader: SCHED_FIFO (CAP_SYS_NICE) and
# mlockall (CAP_IPC_LOCK). Read from the file's security.capability
# attribute (struct vfs_cap_data), so no external getcap tool is needed.
CAP_IPC_LOCK, CAP_SYS_NICE = 14, 23
VFS_CAP_FLAGS_EFFECTIVE = 0x000001

def check_reader_caps(path):
    """Returns a warning text if cw_reader will run without real-time
    priority / memory locking, None if it is fine."""
    if os.geteuid() == 0:
        return None                               # running as root: all allowed
    fix = f"fix (once, repeat after each recompile): sudo setcap cap_sys_nice,cap_ipc_lock=ep {path}"
    try:
        data = os.getxattr(path, "security.capability")
    except (OSError, AttributeError):
        data = b""
    if len(data) < 12:
        return ("cw_reader has no capabilities set - it will run without real-time priority "
                "(SCHED_FIFO) and memory locking; timestamps less precise.\n"
                f"           {fix}")
    magic, permitted = int.from_bytes(data[0:4], "little"), int.from_bytes(data[4:8], "little")
    missing = [name for bit, name in ((CAP_SYS_NICE, "cap_sys_nice"), (CAP_IPC_LOCK, "cap_ipc_lock"))
               if not permitted >> bit & 1]
    if not magic & VFS_CAP_FLAGS_EFFECTIVE:
        missing.append("effective flag (=ep)")
    if missing:
        return (f"cw_reader capabilities incomplete (missing: {', '.join(missing)}) - "
                "real-time priority / memory locking may not work.\n"
                f"           {fix}")
    return None

def estimate(steps):
    """Adds 'est_move' to each step; returns total expected time [s]."""
    pos, total = START_ANGLE_GUESS, 0.0
    for st in steps:
        st["est_move"] = MOVE_OVERHEAD + abs(st["angle"] - pos) * SEC_PER_DEG
        pos = st["angle"]
        total += st["est_move"] + st["seconds"] + READOUT_OVERHEAD
    return total

#------------------------------------------------------------------------
# Running the programs

CURRENT = {"proc": None}   # running child process (used by the web server's Stop button)

def run_command(cmd, cwd, prefix):
    """Run cmd, copy its output (stdout+stderr) line by line to screen and log.
    Returns (exit_code, output_lines, interrupted)."""
    lines, interrupted = [], False
    proc = subprocess.Popen(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            universal_newlines=True, bufsize=1)
    CURRENT["proc"] = proc
    try:
        for line in proc.stdout:
            line = line.rstrip("\n")
            lines.append(line)
            out(f"   {prefix} {line}")
        proc.wait()
    except KeyboardInterrupt:
        # Ctrl-C reaches the child too (same process group) - let it finish cleanly
        interrupted = True
        out(f"   {prefix} Ctrl-C: waiting for the program to stop ...")
        try:
            for line in proc.stdout:
                line = line.rstrip("\n")
                lines.append(line)
                out(f"   {prefix} {line}")
            proc.wait(timeout=15)
        except (KeyboardInterrupt, subprocess.TimeoutExpired):
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
    CURRENT["proc"] = None
    return proc.returncode, lines, interrupted

POS_RE = re.compile(r"position after motor off\s+(-?\d+\.\d+)")
INIT_RE = re.compile(r"initial position\s+(-?\d+\.\d+)")
TOTAL_RE = re.compile(r"total=(\d+)")
STATUS_RE = re.compile(r"^\[(\d+):(\d\d):(\d\d)\] events:")   # cw_reader counter line, time since start
OPENED_RE = re.compile(r"only (\d+) of (\d+) detectors opened")
NOTREADY_RE = re.compile(r"detector\(s\)([\d ]+) not ready")
JOINED_RE = re.compile(r"Detector (\d+) .* joined ([\d.]+) s after the start")
GAPS_RE = re.compile(r"^Gaps without data > (\d+) s: (.*)$")      # cw_reader >= 20261117b
CURRENT_RE = re.compile(r"current position\s+(-?\d+\.\d+)")      # motor_control.py read

def move_motor(setup, angle):
    """One call of motor_control.py <angle>.
    Returns (exit_code, initial_position, position_after_motor_off, interrupted);
    positions are None if not reported (e.g. encoder error)."""
    cmd = [setup["PYTHON"], "-u", setup["MOTOR_CONTROL"], f"{angle:.2f}".rstrip("0").rstrip(".")]
    code, lines, intr = run_command(cmd, os.path.dirname(setup["MOTOR_CONTROL"]), "[motor]")
    p_init = pos = None
    for l in lines:
        m = INIT_RE.search(l)
        if m:
            p_init = float(m.group(1))
        m = POS_RE.search(l)
        if m:
            pos = float(m.group(1))
    return code, p_init, pos, intr

def do_position(st, setup):
    """Move the mount to st['angle'].
    1. normal move (motor_control.py checks and corrects while the motor is on);
    2. if the position after motor off is more than POSITION_TOLERANCE off (e.g. the
       cable torque turned the mount back), up to MAX_CORRECTIONS corrections, each
       approaching the target from the other side: OVERSHOOT deg past the target in
       the direction of the first move, then back to the target;
    3. data are taken at the reached position if it is within POSITION_LIMIT,
       otherwise the plan is stopped.
    Returns (go_on, interrupted)."""
    target = st["angle"]
    t0 = time.time()
    code, p_init, pos, intr = move_motor(setup, target)
    st["reached"] = pos
    if intr:
        st["status"] = "interrupted during positioning"
        return False, True
    if pos is None:
        st["status"] = f"positioning failed (exit code {code}, position unknown)"
        return False, False

    # direction of the first move (+1 = towards larger angles); if the mount was
    # already at the target, the direction in which the error has to be removed
    if p_init is not None and abs(target - p_init) > POSITION_TOLERANCE:
        direction = 1 if target > p_init else -1
    else:
        direction = 1 if target > pos else -1
    via = max(-MOTOR_LIMIT, min(MOTOR_LIMIT, target + direction * OVERSHOOT))

    n_corr = 0
    first_err = pos - target
    while abs(pos - target) > POSITION_TOLERANCE and n_corr < MAX_CORRECTIONS:
        n_corr += 1
        out(f"position {pos:.2f} deg is {pos - target:+.2f} deg from the target - correction "
            f"{n_corr}/{MAX_CORRECTIONS}: via {via:g} deg, then back to {target:g} deg", stamp=True)
        code, _, p_via, intr = move_motor(setup, via)
        if not intr and p_via is not None:
            code, _, p_new, intr = move_motor(setup, target)
        else:
            p_new = None
        if intr:
            st["reached"] = pos
            st["status"] = "interrupted during positioning"
            return False, True
        if p_new is None:
            st["reached"] = None
            st["status"] = f"positioning failed during correction {n_corr} (position unknown)"
            return False, False
        pos = p_new
        st["reached"] = pos
    st["move_time"] = time.time() - t0

    dev = pos - target
    if n_corr:
        st["notes"].append(f"positioning: first {first_err:+.2f} deg, after {n_corr} correction(s) "
                           f"{dev:+.2f} deg")
        out(f"position after {n_corr} correction(s): {pos:.2f} deg ({dev:+.2f} deg from the target)",
            stamp=True)
    if abs(dev) > POSITION_LIMIT:
        st["status"] = (f"positioning failed: {pos:.2f} deg, {dev:+.2f} deg from target "
                        f"after {n_corr} correction(s) (limit {POSITION_LIMIT:g} deg)")
        return False, False
    if abs(dev) > POSITION_TOLERANCE:
        st["notes"].append(f"data taken at {pos:.2f} deg, {dev:+.2f} deg from the target")
        out(f"WARNING: target not reached within {POSITION_TOLERANCE} deg - data taken at "
            f"{pos:.2f} deg (actual angle logged)", stamp=True)
    return True, False

def read_mount(setup):
    """Position of the mount without moving it (motor_control.py read).
    Returns (position or None, interrupted)."""
    cmd = [setup["PYTHON"], "-u", setup["MOTOR_CONTROL"], "read"]
    code, lines, intr = run_command(cmd, os.path.dirname(setup["MOTOR_CONTROL"]), "[motor]")
    pos = None
    for l in lines:
        m = CURRENT_RE.search(l)
        if m:
            pos = float(m.group(1))
    return pos, intr

def check_after_readout(st, setup):
    """Read the position after data taking and compare it with the position at the
    start (st['reached']). Data stay valid; a change > POSITION_TOLERANCE or an
    unknown position is reported (WARNING line + note in the summary).
    Returns interrupted."""
    pos, intr = read_mount(setup)
    st["end_pos"] = pos
    if intr:
        return True
    if pos is None:
        st["notes"].append("position after data taking unknown (encoder error)")
        out("WARNING: position after data taking could not be read", stamp=True)
        return False
    change = pos - st["reached"]
    out(f"position after data taking {pos:.2f} deg ({change:+.2f} deg during data taking)", stamp=True)
    if abs(change) > POSITION_TOLERANCE:
        st["notes"].append(f"mount moved {change:+.2f} deg during data taking "
                           f"({st['reached']:.2f} -> {pos:.2f} deg)")
        out(f"WARNING: mount moved {change:+.2f} deg during data taking "
            f"({st['reached']:.2f} -> {pos:.2f} deg, tolerance {POSITION_TOLERANCE} deg)", stamp=True)
    return False

def do_readout(st, setup):
    """Collect data. Returns (go_on, interrupted)."""
    if os.path.lexists(st["path"]):        # cw_reader appends - never mix data
        st["status"] = "output file appeared before the readout - not started"
        return False, False
    cmd = [setup["CW_READER"], st["path"], fmt_seconds_arg(st["seconds"])]
    t0 = time.time()
    code, lines, intr = run_command(cmd, setup["CW_READER_DIR"], "[readout]")
    st["read_time"] = time.time() - t0      # wall time, incl. waiting for the detector headers
    for l in lines:
        m = STATUS_RE.match(l)
        if m:   # readout time as counted by cw_reader (from all detectors ready); last line wins
            st["read_time"] = int(m.group(1)) * 3600 + int(m.group(2)) * 60 + int(m.group(3))
        m = TOTAL_RE.search(l)
        if m:
            st["events"] = int(m.group(1))
        m = OPENED_RE.search(l)
        if m:
            st["notes"].append(f"only {m.group(1)} of {m.group(2)} detectors")
        m = NOTREADY_RE.search(l)
        if m:
            st["notes"].append(f"not ready at start: det {m.group(1).strip()}")
        m = JOINED_RE.search(l)
        if m:
            st["notes"].append(f"det {m.group(1)} joined after {m.group(2)} s")
        m = GAPS_RE.match(l)
        if m and m.group(2) != "none":
            st["notes"].append(f"gaps > {m.group(1)} s: {m.group(2)}")
    if intr:
        st["status"] = "interrupted during readout (data file kept)"
        return False, True
    if code != 0:
        st["status"] = f"readout failed (exit code {code})"
        return False, False
    if st["read_time"] + 1 < MIN_TIME_FRACTION * st["seconds"]:
        st["status"] = (f"readout ended early after {fmt_time(st['read_time'])} "
                        f"(detectors disconnected?)")
        return False, False
    if not os.path.exists(st["path"]):
        st["status"] = "readout finished but no output file"
        return False, False
    if st.get("events", 0) == 0:
        st["notes"].append("no events recorded")
    st["status"] = "OK"
    if check_after_readout(st, setup):
        st["status"] = "OK, interrupted while reading the position after data taking"
        return False, True
    return True, False

#------------------------------------------------------------------------
# Summary

def summary(steps, t_start, t_end, interrupted):
    out()
    out("=" * 78)
    out("SUMMARY")
    out(f"  started  {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(t_start))}")
    out(f"  finished {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(t_end))}"
        f"   (total {fmt_time(t_end - t_start)})")
    out()
    wname = max(len(st["name"]) for st in steps)
    out(f"  {'#':>2} {'line':>4} {'target':>7} {'reached':>8} {'time':>8} {'measured':>9} "
        f"{'events':>8}  {'file':<{wname}}  status")
    done = 0
    for i, st in enumerate(steps, 1):
        reached = f"{st['reached']:8.2f}" if st.get("reached") is not None else f"{'-':>8}"
        meas = fmt_time(st["read_time"]) if "read_time" in st else "-"
        ev = str(st["events"]) if "events" in st else "-"
        status = st.get("status", "not done")
        if status == "OK":
            done += 1
        notes = f" ({'; '.join(st['notes'])})" if st.get("notes") else ""
        out(f"  {i:2d} {st['line']:4d} {st['angle']:7.2f} {reached} {fmt_time(st['seconds']):>8} "
            f"{meas:>9} {ev:>8}  {st['name']:<{wname}}  {status}{notes}")
    out()
    total_data = sum(st.get("read_time", 0) for st in steps)
    out(f"  observations completed: {done} of {len(steps)};  data taking time {fmt_time(total_data)}")
    if done == len(steps):
        out("  PLAN COMPLETED")
    elif interrupted:
        out("  PLAN INTERRUPTED BY USER (Ctrl-C)")
    else:
        out("  PLAN STOPPED BECAUSE OF AN ERROR - see status above")
    out("=" * 78)
    return 0 if done == len(steps) else 1

#------------------------------------------------------------------------
# Main

def main():
    if len(sys.argv) != 2 or sys.argv[1] in ("-h", "--help"):
        print(f"usage: {os.path.basename(sys.argv[0])} <plan_file>\n")
        print("plan file lines:  <angle [deg]>  <time, e.g. 30s 15m 2h 1h30m>  <output_file>")
        return 2
    plan_path = sys.argv[1]
    if not os.path.isfile(plan_path):
        print(f"ERROR: plan file '{plan_path}' not found")
        return 2

    # 1. checks
    steps, errors = check_plan(plan_path)
    setup, setup_errors = resolve_setup()
    if errors:
        print(f"ERROR: plan file '{plan_path}' is not correct:")
        for e in errors:
            print(f"   {e}")
        print("Nothing was done. Correct the plan and start again.")
        return 2
    if setup_errors:
        print("ERROR: scheduler setup problem:")
        for e in setup_errors:
            print(f"   {e}")
        print("Nothing was done. Check SETUP at the top of cw_scheduler.py.")
        return 2

    # 2. plan overview and time estimate
    total = estimate(steps)
    plan_dir = os.path.dirname(os.path.abspath(plan_path))
    print(f"Plan '{plan_path}' is correct: {len(steps)} observation(s), "
          f"output directory {plan_dir}")
    print()
    print(f"  {'#':>2} {'angle':>7} {'time':>9}  {'est. total':>10}  file")
    for i, st in enumerate(steps, 1):
        print(f"  {i:2d} {st['angle']:7.2f} {fmt_time(st['seconds']):>9}  "
              f"{fmt_time(st['est_move'] + st['seconds'] + READOUT_OVERHEAD):>10}  {st['name']}")
    data_time = sum(st["seconds"] for st in steps)
    print()
    print(f"  data taking time:      {fmt_time(data_time)}")
    print(f"  positioning (approx.): {fmt_time(total - data_time)}")
    print(f"  EXPECTED TOTAL TIME:   {fmt_time(total)}   "
          f"(finish about {time.strftime('%a %H:%M', time.localtime(time.time() + total))})")
    for w in setup["warnings"]:
        print(f"\n  WARNING: {w}")
    print()

    # 3. confirmation
    try:
        ans = input("Start the plan? [y/N]: ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        ans = ""
        print()
    if ans not in ("y", "yes"):
        print("Plan not started.")
        return 2

    # 4. execution
    stamp = time.strftime("%Y%m%d_%H%M%S")
    base = os.path.splitext(os.path.basename(plan_path))[0]
    log_path = os.path.join(plan_dir, f"{base}_{stamp}.log")
    out.open_log(log_path)
    out(f"Plan {os.path.abspath(plan_path)} started {time.strftime('%Y-%m-%d %H:%M:%S')}, "
        f"log {log_path}")
    out(f"Expected total time {fmt_time(total)}")
    for w in setup["warnings"]:
        out(f"WARNING: {w}")
    for st in steps:
        st["notes"] = []

    t_start = time.time()
    interrupted = False
    try:
        for i, st in enumerate(steps, 1):
            out()
            out(f"--- observation {i}/{len(steps)} (plan line {st['line']}): angle {st['angle']:g} deg, "
                f"time {fmt_time(st['seconds'])}, file {st['name']}")
            out(f"positioning to {st['angle']:g} deg", stamp=True)
            go, interrupted = do_position(st, setup)
            if not go:
                out(f"STOP: {st['status']}", stamp=True)
                break
            out(f"position {st['reached']:.2f} deg; data taking for {fmt_time(st['seconds'])}, "
                f"until about {time.strftime('%H:%M:%S', time.localtime(time.time() + st['seconds']))}",
                stamp=True)
            go, interrupted = do_readout(st, setup)
            if not go:
                out(f"STOP: {st['status']}", stamp=True)
                break
            out(f"observation {i} done: {st.get('events', '?')} events", stamp=True)
    except KeyboardInterrupt:          # Ctrl-C between two programs
        interrupted = True
        out("STOP: interrupted by user (Ctrl-C)", stamp=True)
    t_end = time.time()

    code = summary(steps, t_start, t_end, interrupted)
    out(f"Log written to {log_path}")
    out.close()
    return code

if __name__ == "__main__":
    sys.exit(main())
