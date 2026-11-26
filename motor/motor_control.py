#!/usr/bin/env python
# coding: utf-8

import sys
import time
import errno

try:
    import smbus2
    import RPi.GPIO as GPIO

except ImportError as e:
    print(f"Could not load Python module: {e}")
    quit()  
    

#-----------------------------------------------------------------------
# Class for controling step motor

class HR8825:
    def __init__(self, dir_pin, step_pin, enable_pin, mode_pins):
        self.dir_pin = dir_pin
        self.step_pin = step_pin
        self.enable_pin = enable_pin
        self.mode_pins = mode_pins
# GPIO settings
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.enable_pin, GPIO.OUT)
        for pin in self.mode_pins:
            GPIO.setup(pin, GPIO.OUT)
    def digital_write(self, pin, value):
        GPIO.output(pin, value)
    def Stop(self):
        self.digital_write(self.enable_pin, 0)
    def SetMicroStep(self, mode, stepformat):
        microstep = {'fullstep': (0, 0, 0),
                     'halfstep': (1, 0, 0),
                     '1/4step': (0, 1, 0),
                     '1/8step': (1, 1, 0),
                     '1/16step': (0, 0, 1),
                     '1/32step': (1, 0, 1)}
        if (mode == 'softward'):
            self.digital_write(self.mode_pins, microstep[stepformat])
    def move(self, steps, cw_ccw=1, step_delay=0.025):
        GPIO.output(self.enable_pin, GPIO.HIGH)   # Corrected from LOW
        GPIO.output(self.dir_pin, GPIO.HIGH if cw_ccw == -1 else GPIO.LOW)
        delay = step_delay
        for _ in range(steps):
            start = time.perf_counter()
            GPIO.output(self.step_pin, GPIO.HIGH)
            while time.perf_counter() - start < delay:
                pass
            GPIO.output(self.step_pin, GPIO.LOW)
            while time.perf_counter() - start < 2 * delay:
                pass
        GPIO.output(self.enable_pin, GPIO.LOW)     # Corrected from High
    def TurnStep(self, Dir, steps, stepdelay=0.025):
        if (Dir == 'forward'):
#            self.digital_write(self.enable_pin, 1)
            self.digital_write(self.dir_pin, 0)
        elif (Dir == 'backward'):
#            self.digital_write(self.enable_pin, 1)
            self.digital_write(self.dir_pin, 1)
        else:
#            self.digital_write(self.enable_pin, 0)  # Is it needed ?
            return
        if (steps == 0):
            return
        for i in range(steps):
            self.digital_write(self.step_pin, True)
            time.sleep(stepdelay)
            self.digital_write(self.step_pin, False)
            time.sleep(stepdelay)
#        self.digital_write(self.enable_pin, 0)    # Should be added ?
    def StopMotor(self):
        GPIO.output(self.enable_pin, GPIO.LOW)
    def StartMotor(self):
        GPIO.output(self.enable_pin, GPIO.HIGH)

#--------------------------------------------------------------------
# Instance of motor controler

motor = HR8825(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27))
motor.SetMicroStep('softward', 'fullstep')
if not (len(sys.argv) == 2 and sys.argv[1].lower() == "read"):   # read-only mode: motor not used
    time.sleep(2)

#=======================================
# Encoder configuration

AS5600_ADDR = 0x36
ANGLE_REG = 0x0E

# encoder data read from I2C bus

bus = smbus2.SMBus(1)

#----------------------------------------- 
# Encoder reading

def read_angle():
    try:
    	raw_data = bus.read_i2c_block_data(AS5600_ADDR, ANGLE_REG, 2)
    	angle = (raw_data[0] << 8) | raw_data[1]
    	angle = angle & 0x0FFF
#    	angle = angle / 4096.0 * 360.0
    	return angle
    except OSError as exc:
        if exc.errno == errno.EREMOTEIO:   # 121
            print("Device not responding - check wiring and address")
        else:
            print(f"I2C error: {exc}")
        return None


#------------------------------------------------------------------------
# Conversion of encoder reading to absolute mount angle
#
# angle = -(raw - ENC_ZERO) * 360/4096 + ENC_CORR_AMP * sin(2 * 2*pi*raw/4096 + ENC_CORR_PHASE)
#
# Gives the angle in degrees as read by the electronic level (same sign
# convention as the level, which counts opposite to the encoder).
# raw - ENC_ZERO is wrapped into [-2048, 2048), i.e. +-180 deg around zero.
# The sine term (period 180 deg) corrects the encoder non-linearity
# (typical of a tilted magnet / elliptic field; a 360-deg term fitted much worse).
# Constants fitted to core_MOT_raw_20261030.dat (51 points, -86..+86 deg),
# hysteresis neglected: residual rms 0.17 deg, max 0.38 deg.

import math

ENC_COUNTS = 4096
ENC_ZERO = 3773.3            # encoder reading at level angle 0 (without correction term)
ENC_CORR_AMP = 1.668         # deg
ENC_CORR_PHASE = -141.7      # deg

def convert_angle(raw):
    d = (raw - ENC_ZERO + ENC_COUNTS / 2) % ENC_COUNTS - ENC_COUNTS / 2
    angle = -d * 360.0 / ENC_COUNTS
    angle += ENC_CORR_AMP * math.sin(4 * math.pi * raw / ENC_COUNTS + math.radians(ENC_CORR_PHASE))
    return angle

#------------------------------------------------------------------------
# Mount and positioning parameters

MOTOR_STEPS_PER_REV = 200    # full steps per motor revolution
GEAR_RATIO = 16 / 100        # mount turns per motor turn (16:100 transmission)
DEG_PER_STEP = 360.0 / MOTOR_STEPS_PER_REV * GEAR_RATIO   # 0.288 deg
STEP_SIGN = -1               # sign of the level-angle change for a 'forward' step
STEP_DELAY = 0.025           # s, half period of the step signal (20 steps/s)

ANGLE_TOLERANCE = 0.2        # deg, accepted deviation from the requested angle (motor on);
                             # 0.5 until 25 Nov 2026 - the full +-0.5 was used (random-position test)
OFF_SHIFT_TOLERANCE = 0.5    # deg, change after motor off that gives a WARNING (single-move mode)
N_STABLE_READS = 3           # consecutive reads within tolerance = move completed
READ_INTERVAL = 1.0          # s, time between the checking reads
MAX_CORRECTIONS = 10         # give up after this many correction moves
ANGLE_MIN = -96.0            # deg, allowed range of requested angles (+-96 since 18 Nov 2026:
ANGLE_MAX = 96.0             # room for the scheduler's 5 deg overshoot at +-90 deg; plans stay +-90)
ENC_SAMPLES = 5              # encoder reads per position measurement (median)

#------------------------------------------------------------------------
# Position measurement and movement

def read_position():
    """Mount angle in deg (level convention): median of ENC_SAMPLES encoder reads.
    Returns None if the encoder could not be read."""
    vals = []
    for _ in range(ENC_SAMPLES):
        raw = read_angle()
        if raw is not None:
            vals.append(convert_angle(raw))
        time.sleep(0.002)
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2]

def move_steps(steps):
    """Move by a signed number of motor steps (positive = 'forward')."""
    if steps > 0:
        motor.TurnStep('forward', steps, STEP_DELAY)
    elif steps < 0:
        motor.TurnStep('backward', -steps, STEP_DELAY)

def stamp():
    return time.strftime("%H:%M:%S")

def move_to(target, verbose=True):
    """Move the mount to the target angle and check it.
    Returns (ok, pos): ok is True when N_STABLE_READS consecutive reads,
    READ_INTERVAL apart, are within ANGLE_TOLERANCE of the target; pos is the
    last position read (None if the encoder could not be read).
    verbose=False: only failures are printed."""
    def log(msg):
        if verbose:
            print(f"{stamp()} : {msg}")
    motor.StartMotor()
    pos = None
    for attempt in range(MAX_CORRECTIONS + 1):
        pos = read_position()
        if pos is None:
            print(f"{stamp()} : ERROR: encoder not readable - movement stopped")
            return False, None
        err = target - pos
        steps = round(err / (STEP_SIGN * DEG_PER_STEP))
        if steps != 0:
            what = "move" if attempt == 0 else f"correction {attempt}"
            log(f"{what}: position {pos:7.2f}, error {err:+6.2f} deg -> {steps:+d} steps")
            move_steps(steps)
        # check: N_STABLE_READS consecutive reads within tolerance
        n_good = 0
        while n_good < N_STABLE_READS:
            time.sleep(READ_INTERVAL)
            pos = read_position()
            if pos is None or abs(pos - target) > ANGLE_TOLERANCE:
                break
            n_good += 1
            log(f"check {n_good}/{N_STABLE_READS}: position {pos:7.2f} deg (target {target:.2f})")
        if n_good == N_STABLE_READS:
            log(f"position reached: {pos:7.2f} deg (target {target:.2f}, "
                f"{'no corrections' if attempt == 0 else f'{attempt} correction(s)'})")
            return True, pos
        if pos is not None:
            log(f"check failed: position {pos:7.2f} deg (target {target:.2f})")
    print(f"{stamp()} : WARNING: target {target:.2f} deg not reached after {MAX_CORRECTIONS} corrections")
    return False, pos

#------------------------------------------------------------------------
# Single-move mode: motor_control.py <angle>
#
# Moves to the angle (with checks/corrections), switches the motor off,
# reads the position N_STABLE_READS times OFF_READ_INTERVAL apart and exits.
# Prints: initial position, position after movement, position after motor
# off, plus a warning if the position moved by more than OFF_SHIFT_TOLERANCE
# after switching off. Exit code 0 = OK, 1 = problem, 2 = bad argument.

OFF_READ_INTERVAL = 0.5      # s, time between reads after the motor is switched off

def fmt(pos):
    return "  n/a  " if pos is None else f"{pos:7.2f}"

def single_move(arg):
    try:
        target = float(arg)
    except ValueError:
        print(f"'{arg}' is not a number")
        return 2
    if not ANGLE_MIN <= target <= ANGLE_MAX:
        print(f"angle {target} outside allowed range {ANGLE_MIN}..{ANGLE_MAX} deg")
        return 2
    pos0 = read_position()
    print(f"{stamp()} : initial position         {fmt(pos0)} deg")
    if pos0 is None:
        print(f"{stamp()} : ERROR: encoder not readable - no movement")
        return 1
    ok, pos1 = move_to(target, verbose=False)
    print(f"{stamp()} : position after movement  {fmt(pos1)} deg (target {target:.2f})")
    motor.StopMotor()
    off = []
    for _ in range(N_STABLE_READS):
        time.sleep(OFF_READ_INTERVAL)
        off.append(read_position())
    pos2 = off[-1]
    print(f"{stamp()} : position after motor off {fmt(pos2)} deg (reads: {' '.join(fmt(p).strip() for p in off)})")
    valid = [p for p in off if p is not None]
    if pos1 is not None and valid:
        shift = max(valid, key=lambda p: abs(p - pos1)) - pos1
        if abs(shift) > OFF_SHIFT_TOLERANCE:
            print(f"{stamp()} : WARNING: position changed by {shift:+.2f} deg after motor off "
                  f"(tolerance {OFF_SHIFT_TOLERANCE} deg)")
            ok = False
    elif not valid:
        print(f"{stamp()} : ERROR: encoder not readable after motor off")
        ok = False
    return 0 if ok else 1

#------------------------------------------------------------------------
# Read-only mode: motor_control.py read
#
# Reads the position without touching the motor (it stays off) and prints
#   HH:MM:SS : current position           xx.xx deg
# Exit code 0 = OK, 1 = encoder not readable. Used by the scheduler after
# each readout to check that the mount did not move during data taking.

def read_only():
    pos = read_position()
    print(f"{stamp()} : current position         {fmt(pos)} deg")
    if pos is None:
        print(f"{stamp()} : ERROR: encoder not readable")
        return 1
    return 0

#------------------------------------------------------------------------
# Main

def interactive():
    print("Motor and encoder started on", time.strftime("%A, %B %d, %Y %H:%M:%S"))
    print(f"Gear ratio {GEAR_RATIO:.3f} -> {DEG_PER_STEP:.3f} deg/step; tolerance {ANGLE_TOLERANCE} deg, "
          f"{N_STABLE_READS} reads {READ_INTERVAL} s apart; allowed range {ANGLE_MIN}..{ANGLE_MAX} deg")
    while True:
        raw = read_angle()
        if raw is None:
            print(f"{stamp()} : encoder not readable")
        else:
            print(f"{stamp()} : encoder {raw:5d}   angle {convert_angle(raw):7.2f} deg")

        instr = input("Angle to set [deg] (Enter = read position and release motor, q = quit): ").strip()
        if instr.lower() in ("q", "quit", "exit"):
            break
        if instr == "":
            motor.StopMotor()
            time.sleep(0.5)
            continue
        try:
            target = float(instr)
        except ValueError:
            print(f"'{instr}' is not a number")
            continue
        if not ANGLE_MIN <= target <= ANGLE_MAX:
            print(f"angle {target} outside allowed range {ANGLE_MIN}..{ANGLE_MAX} deg")
            continue
        move_to(target)
    print("Exiting...")

if len(sys.argv) > 2:
    print(f"usage: {sys.argv[0]} [angle | read]   (no argument = interactive mode)")
    GPIO.cleanup(); bus.close()
    sys.exit(2)

status = 0
try:
    if len(sys.argv) == 2 and sys.argv[1].lower() == "read":
        status = read_only()
    elif len(sys.argv) == 2:
        status = single_move(sys.argv[1])
    else:
        interactive()
except KeyboardInterrupt:
    print("Interrupted")
    status = 1
finally:
    motor.StopMotor()
    bus.close()
    GPIO.cleanup()
sys.exit(status)
