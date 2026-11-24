# Installation on the Raspberry Pi

Tested with Raspberry Pi OS (Debian 12, 32-bit, GCC 12, Python 3.11).
In the examples the lab user is `doroba` and the repository is cloned to `~/CW`;
adjust names and paths if needed.

## 1. System packages and permissions

```
sudo apt update
sudo apt install git g++ libcap2-bin tcsh python3-rpi.gpio python3-smbus2
sudo raspi-config nonint do_i2c 0          # enable I2C (or: raspi-config → Interface Options → I2C)
sudo usermod -aG gpio,i2c,dialout doroba   # GPIO + I2C (motor), serial ports (detectors)
```

Log out and in again for the new groups to take effect.

Note: on a Raspberry Pi 5 the classic `RPi.GPIO` does not work; install the drop-in
replacement `python3-rpi-lgpio` instead.

## 2. Get the code

```
cd ~
git clone git@github.com:zarnecki/cosmicwatch.git CW
```

Layout:

```
~/CW/readout/    cw_reader.cpp  make_cw_dev
~/CW/motor/      motor_control.py
~/CW/scheduler/  cw_scheduler.py  example_plan.txt
~/CW/server/     cw_server.py  cw_server.service
```

## 3. Readout

```
cd ~/CW/readout
g++ -std=c++17 -O2 -Wall -Wextra -Wno-psabi -pthread cw_reader.cpp -o cw_reader
sudo setcap cap_sys_nice,cap_ipc_lock=ep cw_reader
```

The capabilities give `cw_reader` real-time priority and locked memory without running it
as root. Repeat `setcap` after every rebuild (copying or rebuilding the binary removes them).
In a student lab the binary should not be writable by the students (e.g.
`sudo chown root:root cw_reader`).

Detector links (with all 8 detectors connected):

```
ls -l /dev/serial/by-id/      # compare the FTDI serial numbers with those in make_cw_dev
./make_cw_dev                 # creates dev/CW0 .. dev/CW7
```

Quick test (should list 8 detector names and count events):

```
./cw_reader - 60
```

## 4. Motor

Quick test in interactive mode (motor powered, mount free to move):

```
cd ~/CW/motor
python3 motor_control.py      # Enter = read position, a number = move there, q = quit
```

## 5. Scheduler

The scheduler looks for `motor_control.py`, `cw_reader` and `dev/` in its own directory
unless told otherwise. With the repository layout set the paths once, e.g. in `~/.bashrc`:

```
export MOTOR_CONTROL=../motor/motor_control.py
export CW_READER=../readout/cw_reader
export CW_READER_DIR=../readout
```

(relative to `~/CW/scheduler`; absolute paths work as well).
Test with a short plan in a data directory, e.g.:

```
mkdir -p ~/cw_data/test && cd ~/cw_data/test
printf "0 30s test_000.txt\n30 30s test_p30.txt\n" > plan.txt
python3 ~/CW/scheduler/cw_scheduler.py plan.txt
```

The set-up check at the start reports missing programs, detectors or `cw_reader`
capabilities.

## 6. Web server

Test by hand:

```
CW_DATA_ROOT=~/cw_data python3 ~/CW/server/cw_server.py 8080
```

and open `http://<pi-address>:8080/`. (The environment variables of step 5 must be set in
this shell too.)

Permanent installation as a systemd service: edit `~/CW/server/cw_server.service` —
`User`, `WorkingDirectory`, `ExecStart`, `CW_DATA_ROOT` — and add the program paths:

```
Environment=MOTOR_CONTROL=/home/doroba/CW/motor/motor_control.py
Environment=CW_READER=/home/doroba/CW/readout/cw_reader
Environment=CW_READER_DIR=/home/doroba/CW/readout
```

Then:

```
sudo cp ~/CW/server/cw_server.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now cw_server
systemctl status cw_server
```

The server runs as the lab user, never as root. Data are written to
`CW_DATA_ROOT` (default `~/cw_data`).

## 7. Keeping the working copy clean

The binary and the detector links are created inside the repository. To keep them out of
`git status` without committing a `.gitignore`:

```
cd ~/CW
printf "readout/cw_reader\nreadout/dev/\n" >> .git/info/exclude
```

## Updating

```
cd ~/CW
git pull
cd readout && g++ -std=c++17 -O2 -Wall -Wextra -Wno-psabi -pthread cw_reader.cpp -o cw_reader \
  && sudo setcap cap_sys_nice,cap_ipc_lock=ep cw_reader
sudo systemctl restart cw_server       # not while a plan is running – it would be stopped
```
