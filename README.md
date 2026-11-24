# CosmicWatch DAQ

Data acquisition (DAQ) system for an advanced physics laboratory experiment based on
[CosmicWatch](http://www.cosmicwatch.lns.mit.edu/) detectors.
The goal of the experiment is to measure the total flux of secondary cosmic rays and its
angular distribution.

## Experimental set-up

- 8 CosmicWatch detectors (scintillator + SiPM + Arduino Nano), each connected to the
  Raspberry Pi by USB (serial line, 9600 baud). Each detector prints one text line per event.
- The detectors are mounted on a rotating mount driven by a stepper motor (HR8825 driver,
  16:100 transmission, 0.288°/step). The mount angle is measured by an AS5600 magnetic
  encoder on I2C (absolute, calibrated against an electronic level).
- Everything is controlled by a Raspberry Pi (Raspberry Pi OS, Python 3, GCC).

## DAQ structure

```
              browser (lab network)
                     │ HTTP
            ┌────────▼────────┐
            │  server/        │  cw_server.py  – web front end for students
            │                 │  (imports cw_scheduler.py as a module)
            └────────┬────────┘
            ┌────────▼────────┐
            │  scheduler/     │  cw_scheduler.py – runs an observation plan:
            │                 │  for every line "angle  time  file" ...
            └───┬─────────┬───┘
     1. position│         │2. take data
   ┌────────────▼──┐   ┌──▼─────────────┐
   │  motor/       │   │  readout/      │
   │ motor_control │   │  cw_reader     │  8 detectors (dev/CW0..CW7)
   │ .py <angle>   │   │ <file> <sec>   │  → one data file per observation
   └───────────────┘   └────────────────┘
     GPIO + I2C            USB serial
```

| Component | Directory | Program | Role |
|-----------|-----------|---------|------|
| Readout   | [`readout/`](readout/README.md)     | `cw_reader` (C++)        | Reads all 8 detectors in parallel, time-stamps every event, writes one data file. |
| Motor     | [`motor/`](motor/README.md)         | `motor_control.py`       | Turns the mount to a requested angle (closed loop with the encoder). |
| Scheduler | [`scheduler/`](scheduler/README.md) | `cw_scheduler.py`        | Executes an observation plan (sequence of angle / time / file) from the command line, with checks, corrections and a log. |
| Server    | [`server/`](server/README.md)       | `cw_server.py`           | Web interface: students prepare and start a plan, follow its progress and download the data. |

Each component can also be used on its own (e.g. `cw_reader` for a long run at a fixed
angle, `motor_control.py` interactively for tests of the mount).
The scheduler and the server run the motor and readout programs as separate processes,
so data taking (real-time priority in `cw_reader`) is not disturbed by Python or by page
requests.

## Typical use

- **Students:** open `http://<raspberry-pi-address>:8080/` in a browser, enter name and
  initials, add observations, press *Start plan*, download the `.tar.gz` with all data
  at the end. See [`server/README.md`](server/README.md).
- **Command line:** write a plan file and run `python3 cw_scheduler.py plan.txt`.
  See [`scheduler/README.md`](scheduler/README.md).

## Installation

See [INSTALLATION.md](INSTALLATION.md).

## Data format

One line per event:

```
<detector 0-7> <time since start [µs]> <Event> <Ardn_time [ms]> <ADC> <SiPM [mV]> <Deadtime [ms]> <Temp [°C]>
```

The first two fields are added by `cw_reader` (host time, CLOCK_MONOTONIC); the rest is
the line sent by the detector firmware. Details in [`readout/README.md`](readout/README.md).
