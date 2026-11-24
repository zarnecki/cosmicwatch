# Server – `cw_server.py`

Web front end for the scheduler: students prepare and run an observation plan from a
browser instead of the Raspberry Pi command line. Python standard library only.

## Usage

```
python3 cw_server.py [port]          # default port 8080
```

then open `http://<raspberry-pi-address>:8080/` in any browser on the lab network.
For permanent use the server is started at boot by systemd (`cw_server.service`, see
below).

## Work flow in the browser

1. **Start page:** name, surname and initials → a new data directory
   `<CW_DATA_ROOT>/<YYYYMMDD_HHMM>_<INITIALS>` is created (with `info.txt`).
2. **Plan page:** add observations (angle, time, file name) one by one or paste several
   plan lines; each entry is checked immediately with the scheduler rules; rows can be
   deleted; the expected total time is shown. Set-up problems (missing programs,
   no detectors) block the *Start plan* button.
   **Disk space check:** the page also shows the expected data size (`CW_MB_PER_HOUR`,
   default 1.4 MB per hour of data taking, including the final `.tar.gz`) and the free disk
   space in `CW_DATA_ROOT`. If the free space is not sufficient, the plan cannot be started
   – free some space (or shorten the plan) and press *Recheck disk space*. The check is
   repeated when *Start plan* is pressed. Below 1 GB of free space a warning is shown.
3. **Start plan:** the plan is written to `plan.txt` and executed in the background
   exactly as by `cw_scheduler.py`; the full output goes to `plan_<date>.log`.
4. **Progress page:** one row per observation (yellow = running, green = done,
   red = failed/stopped), reached angle, measured time, events, notes; finished files are
   download links. The page reloads every 10 min (*Refresh now* for the current state).
   It also shows the free disk space and the space still needed for the remaining
   observations, with an error message if the free space has become too small (the plan is
   not stopped automatically). *Stop plan* ends the current observation cleanly and skips
   the rest.
5. **Finished page:** everything (data files, plan, info, log) packed into
   `<directory>.tar.gz` for download; *New session* returns to step 1.

There is one plan at a time (one detector set-up); every browser sees the same state.
No passwords – meant for the lab network only. `/status.json` returns the state for
scripts and debugging.

## Configuration (environment variables)

| Variable | Default | Meaning |
|----------|---------|---------|
| `CW_SERVER_PORT` | `8080` | port (the command-line argument takes precedence) |
| `CW_DATA_ROOT`   | `~/cw_data` | where the session directories are created |
| `CW_REFRESH`     | `600` | page reload interval while a plan runs [s] |
| `CW_MB_PER_HOUR` | `1.4` | expected disk use per hour of data taking [MB] (~1 MB data + ~40 % for the `.tar.gz`), used by the disk space check |
| `CW_SCHEDULER`   | – | path of `cw_scheduler.py` (default: next to `cw_server.py` or in `../scheduler/`) |

The server imports `cw_scheduler.py` and uses its checks, positioning, readout, log and
summary, so the paths of `motor_control.py`, `cw_reader` and `dev/` are configured as for
the scheduler (`MOTOR_CONTROL`, `CW_READER`, `CW_READER_DIR`, see
[`../scheduler/README.md`](../scheduler/README.md)).

## systemd service

`cw_server.service` starts the server at boot as the normal lab user (not root).
Edit `User`, the paths and the `Environment` lines first, then:

```
sudo cp cw_server.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now cw_server     # start now and at every boot
systemctl status cw_server                # check
journalctl -u cw_server -f                # follow the server output
```

Stopping the service (`sudo systemctl stop cw_server`) ends a running observation cleanly
(SIGINT). The state is kept in memory: after a restart the session is not resumed (its data
directory and log stay in `CW_DATA_ROOT`).
