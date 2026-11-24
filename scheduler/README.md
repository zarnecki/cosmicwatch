# Scheduler – `cw_scheduler.py`

Runs an observation plan: a list of mount angles, observation times and output files.
For every line it moves the mount (`motor_control.py <angle>`) and then takes data
(`cw_reader <file> <seconds>`).

## Plan file

One observation per line: `angle  time  output_file`

```
# angle   time    file
   0      1h      zenith_000.txt
  30      1h30m   tilt_p30.txt
 -30      90m     tilt_m30.txt
```

- `angle` – mount angle in degrees, −90 … 90;
- `time` – with unit `h`, `m` or `s`, combinable (`2h`, `1.5h`, `1h30m`, `45s`), 10 s … 24 h;
- `output_file` – letters, digits, `_ - . +` only, starting with a letter or digit; written
  in the directory of the plan; must not exist yet and must be unique in the plan.
- Empty lines and text after `#` are ignored. See [`example_plan.txt`](example_plan.txt).

## Usage

```
python3 cw_scheduler.py plan.txt
```

Run it as the normal user, **not with sudo** (data and log files would belong to root).

1. The whole plan and the set-up (motor script, `cw_reader`, `dev/CW0..7`, capabilities of
   `cw_reader`) are checked first; all errors are listed with line numbers.
2. A table of the plan, the expected total time and finish time are shown; the plan starts
   after confirmation (`y`).
3. Per observation: positioning, then readout. Everything shown on the screen also goes to
   `<plan>_<YYYYmmdd_HHMMSS>.log` next to the plan.
4. At the end a summary table: target and reached angle, requested and measured time,
   number of events, status and notes (corrections, late detectors, data gaps …).

Ctrl-C stops the current step cleanly (motor off / data file closed) and ends the plan with
the summary. Exit code: 0 = all done, 1 = stopped (error or Ctrl-C), 2 = plan or set-up
error (nothing done).

## Positioning and error handling

- If the position after motor off is more than 0.5° from the target, the mount is moved
  5° past the target and back, i.e. the target is approached from the other side
  (up to 3 corrections).
- Still more than 5° off (or position unknown) → plan stopped. Otherwise data are taken
  at the reached angle, which is written in the log and summary.
- Readout failed, no output file, or shorter than 95 % of the requested time → plan stopped.

## Configuration

Paths of the other programs are set in `SETUP` at the top of the script and can be
overridden by environment variables (relative paths are relative to the scheduler
directory):

| Variable | Default | Meaning |
|----------|---------|---------|
| `MOTOR_CONTROL` | `motor_control.py` | motor positioning script |
| `CW_READER`     | `cw_reader`        | compiled readout program |
| `CW_READER_DIR` | `.`                | directory containing `dev/CW0..CW7` (working directory of `cw_reader`) |
| `PYTHON`        | current interpreter | Python used for `motor_control.py` |

With the repository layout (see [INSTALLATION.md](../INSTALLATION.md)):

```
export MOTOR_CONTROL=../motor/motor_control.py
export CW_READER=../readout/cw_reader
export CW_READER_DIR=../readout
```
