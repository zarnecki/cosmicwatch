# Motor – `motor_control.py`

Turns the rotating detector mount to a requested angle.

## Hardware

- Stepper motor with an HR8825 driver on the Raspberry Pi GPIO (BCM numbering):
  DIR 24, STEP 18, ENABLE 4, mode pins 21/22/27 (full step).
  Transmission 16:100 → 0.288° of the mount per motor step.
- AS5600 magnetic encoder on I2C bus 1 (address 0x36), 12 bit, absolute.
- Angle convention: as shown by the electronic level used for the calibration,
  0° = detectors in the standard (vertical) position. The encoder reading is converted to
  this angle by `convert_angle()` (linear term + sin(2θ) correction for the encoder
  non-linearity; fitted over −86…+86°, residual rms 0.17°).

## How it moves

`move_to(target)`: read the position (median of 5 encoder reads), make the required
number of steps, then read the position every second; the move is done after 3
consecutive reads within ±0.5° of the target, otherwise a correction move is made
(at most 10). Allowed range of targets: ±96° (the scheduler limits plans to ±90° and uses
the extra range for approaching the target from the other side).

## Usage

Requirements: Python 3 with `RPi.GPIO` and `smbus2`, I2C enabled, user in the groups
`gpio` and `i2c` (no sudo needed).

**Interactive mode** (tests, manual positioning):

```
python3 motor_control.py
```

Enter an angle in degrees to move there (the motor stays on and holds the position);
Enter alone = show the position and switch the motor off; `q` = quit.

**Single-move mode** (used by the scheduler):

```
python3 motor_control.py <angle>
```

Moves to the angle, switches the motor **off**, reads the position 3 times and prints:
initial position, position after movement, position after motor off (and a WARNING if
the mount moved by more than 0.5° after the motor was switched off).
Exit code: 0 = OK, 1 = problem (target not reached, shift after motor off, encoder
error, Ctrl-C), 2 = bad argument.

## Known features

- Cable tension can turn the mount back towards 0° when the motor is switched off
  (seen at positive angles, up to a few degrees at 45°). The scheduler detects this and
  corrects it by approaching the target from the other side.
- Beyond ±86° the angle is extrapolated from the calibration.
