# Readout – `cw_reader`

`cw_reader.cpp` reads the 8 CosmicWatch detectors connected by USB, time-stamps every
event and writes all events to one file (or to the terminal).

## How it works

- One `epoll` loop over the 8 serial ports (non-blocking, raw mode) plus a separate
  writer thread; real-time priority (`SCHED_FIFO`) and locked memory (`mlockall`) if the
  binary has the needed capabilities.
- Ports are opened as `dev/CW0` … `dev/CW7` **relative to the working directory**.
  These are symlinks to `/dev/serial/by-id/...`, so the detector numbering does not
  depend on the USB port order. They are created by `make_cw_dev`.
- Every detector sends a header (`###...` lines) and its unique name when the port is
  opened. The header is discarded, the names are printed.
  Time zero ("Readout started") is the moment when **all** detectors have sent their
  name (at most 10 s after opening; detectors not ready by then join later).
- Every event is stamped with the host time (`CLOCK_MONOTONIC`) of the arrival of the
  **first byte** of its line.
- Warning when a detector sends no data for more than 30 s; a summary of such gaps is
  printed at the end.

## Build

```
g++ -std=c++17 -O2 -Wall -Wextra -Wno-psabi -pthread cw_reader.cpp -o cw_reader
sudo setcap cap_sys_nice,cap_ipc_lock=ep cw_reader     # repeat after every build
```

(`-Wno-psabi` hides harmless ABI notes of GCC on 32-bit ARM.)

## Detector links

```
./make_cw_dev        # (tcsh script) creates dev/CW0..dev/CW7 in the current directory
```

The serial numbers of the 8 detectors (FTDI USB-serial IDs) are written in the script;
check them with `ls -l /dev/serial/by-id/` and edit the script if detectors are replaced.

## Usage

```
cw_reader [output_file|-] [seconds] [baud]
```

| Example | Meaning |
|---------|---------|
| `cw_reader`                     | events printed to the terminal, stop with Ctrl-C |
| `cw_reader events.txt`          | events appended to `events.txt`, stop with Ctrl-C (or SIGTERM) |
| `cw_reader events.txt 3600`     | same, stops by itself after 3600 s |
| `cw_reader events.txt 0 115200` | no time limit (0), ports opened at 115200 baud |

Run it from the directory that contains `dev/` (here: `readout/`).
Data go to the file (or stdout); detector names, event counters, warnings and errors go
to stderr. For long manual runs it is convenient to keep the screen output too:

```
./cw_reader run.log 2>&1 | tee -i run.out
```

(`tee -i` ignores Ctrl-C, so only `cw_reader` gets the signal and stops cleanly.)
To stop a run from another terminal: `kill -TERM $(pgrep -f cw_reader)`.

The baud rate (default 9600) must match `Serial.begin()` in the detector firmware.

## Output format

```
<detector> <time_us> <Event> <Ardn_time[ms]> <ADC[0-1023]> <SiPM[mV]> <Deadtime[ms]> <Temp[C]>
```

- `detector` – 0…7 (= `dev/CWn`), `time_us` – host time since the readout start [µs];
- the rest is the detector line, stored unchanged. `Ardn_time` (detector clock, 1 ms
  resolution) is better than the host time for coincidences between detectors
  (host-time jitter ~±5–10 ms, tails up to ~200 ms), but the detector clocks drift and
  must be synchronised offline.
- Lines are not inspected after the detector name, so a detector reset during a run
  stores its header lines as data (remove with `grep -v '#'`).
