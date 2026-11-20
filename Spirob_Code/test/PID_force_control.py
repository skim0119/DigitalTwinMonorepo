"""
Force-control loop for a single stepper motor, built on top of your existing
motors.py (Motors class) from the xbox_controller.py project.

WHAT THIS DOES
--------------
Every `PID_DT` seconds it:
  1. reads the load cell and converts the raw HX711 count to a force (kg)
     using your calibration offset/factor
  2. runs a PID step to compare that force against a target setpoint
  3. converts the PID output (which is in steps/sec, signed) into:
       - a direction (dir_motor_forward / dir_motor_backward)
       - a speed fraction 0..1 of max_rpm (set_motor_speed)

In between PID updates, it keeps calling motors.update([motor_index]) as
fast as possible -- that's the call that actually toggles the PUL pin and
is what makes the motor turn at whatever speed was last set. This mirrors
the same while-loop pattern your xbox_controller.py already uses.
"""

import time
import RPi.GPIO as GPIO
from hx711 import HX711
from motors import Motors

# ----------------------------------------------------------------------
# Motor / pin setup (same values as xbox_controller.py)
# ----------------------------------------------------------------------
PUL = [4, 23, 22, 26]
DIR = [27, 24, 25, 16]
MOTOR_ORIENTATION = [True, True, True, True]

MOTOR_INDEX = 0          # which motor in the list you're controlling

# HX711.__init__() calls GPIO.setup() on the load cell pins, which requires
# GPIO.setmode() to already have been called. Do it here, before anything
# else touches GPIO. If motors.py also calls GPIO.setmode(GPIO.BCM)
# internally, this is harmless (same mode, just set twice).
GPIO.setmode(GPIO.BCM)

motors = Motors(
    PUL,
    DIR,
    MOTOR_ORIENTATION,
    steps_per_rev=1600,
    max_rpm=30,
    pulse_width=10e-6,
    debug=False,          # set True to test without real GPIO hardware
)

MAX_STEPS_PER_SEC = motors.max_rpm / 60 * motors.steps_per_rev

# ----------------------------------------------------------------------
# Load cell setup -- same DOUT/SCK pin map as test_loadcell_thread.py.
# motor0 (MOTOR_INDEX 0) uses DOUT=5 / SCK=6.
# ----------------------------------------------------------------------
DOUT_PINS = [5, 21, 13, 19]
SCK_PINS = [6, 20, 17, 12]
# NOTE: SCK_PINS[2] (17) is the same GPIO as PUL[0] (17). That's fine while
# you're only driving MOTOR_INDEX 0, but it will conflict the moment you
# also wire up motor2's load cell -- move one of them off GPIO17 first.

load_cell = HX711(
    dout_pin=DOUT_PINS[MOTOR_INDEX],
    pd_sck_pin=SCK_PINS[MOTOR_INDEX],
    select_channel='A',
    gain_channel_A=64,
)
load_cell.reset()

# ----------------------------------------------------------------------
# Load cell calibration -- from your 5-point calibration:
#   0 g -> 276320, 330.3 g -> 287887, 370.4 g -> 289903,
#   446.3 g -> 293211, 618.8 g -> 300803  (R^2 = 0.995)
# ----------------------------------------------------------------------
CAL_FACTOR = 39293.8     # counts per kg (regression slope: 39.2938 counts/g)
RAW_OFFSET = 275748    # raw reading at zero load (your direct tare measurement --
                          # the regression's fitted intercept was 275748, close but
                          # this is what you actually measured with nothing on the cell)

_last_good_raw = RAW_OFFSET  # fallback so a bad HX711 read never gets silently treated as 0


def read_raw_load_cell() -> int:
    """
    Reads the HX711 for this motor's channel. Uses a small readings count
    (3) to keep the PID loop responsive -- raise it for less noise at the
    cost of a slower loop, since each reading blocks until the HX711's
    DOUT pin goes low (10-80 Sa/s depending on gain/mode).
    """
    global _last_good_raw
    raw = load_cell.get_raw_data_mean(readings=3)
    if raw is False:
        # bad/timed-out reading -- reuse the last good value instead of
        # letting `False` (== 0 in arithmetic) silently corrupt the force calc
        return _last_good_raw
    _last_good_raw = raw
    return raw


def read_force_kg() -> float:
    raw = read_raw_load_cell()
    return (raw - RAW_OFFSET) / CAL_FACTOR


# ----------------------------------------------------------------------
# PID setup
# ----------------------------------------------------------------------
SETPOINT_FORCE_KG = 0.5   # target tension/force -- change as needed

Kp = 10.0   # steps/sec of output per kg of error -- START HERE, see notes
Ki = 0.0
Kd = 0.0     # leave at 0 until Kp/Ki feel reasonable (load cell noise + D = bad)

PID_DT = 0.02   # trigger threshold only -- actual dt used below is measured,
                # so this doesn't need to exactly match the HX711's real rate

_integral = 0.0
_prev_error = 0.0


def pid_step(setpoint: float, measured: float, dt: float) -> float:
    global _integral, _prev_error

    error = setpoint - measured

    _integral += error * dt
    derivative = (error - _prev_error) / dt if dt > 0 else 0.0
    _prev_error = error

    output = Kp * error + Ki * _integral + Kd * derivative

    # clamp to actuator limits + basic anti-windup (back the integral off
    # by the amount that pushed us past the limit)
    if output > MAX_STEPS_PER_SEC:
        _integral -= error * dt
        output = MAX_STEPS_PER_SEC
    elif output < -MAX_STEPS_PER_SEC:
        _integral -= error * dt
        output = -MAX_STEPS_PER_SEC

    return output  # steps/sec, sign = direction


def apply_pid_output_to_motor(output_steps_per_sec: float):
    """This is the piece that answers 'how do I use the PID output on the motor'."""
    if output_steps_per_sec >= 0:
        motors.dir_motor_forward([MOTOR_INDEX])
    else:
        motors.dir_motor_backward([MOTOR_INDEX])

    speed_fraction = min(abs(output_steps_per_sec) / MAX_STEPS_PER_SEC, 1.0)
    motors.set_motor_speed([MOTOR_INDEX], speed_fraction)


# ----------------------------------------------------------------------
# Main loop
# ----------------------------------------------------------------------
def main():
    motors.start_motor([MOTOR_INDEX])
    last_pid_time = time.time()

    try:
        while True:
            now = time.time()
            if now - last_pid_time >= PID_DT:
                dt = now - last_pid_time
                last_pid_time = now

                measured_force = read_force_kg()
                output = pid_step(SETPOINT_FORCE_KG, measured_force, dt)
                apply_pid_output_to_motor(output)

                # optional: comment out once things are working
                print(f"force={measured_force:.3f}kg  output={output:.1f} steps/s")

            # this is what actually pulses PUL -- must be called continuously
            motors.update([MOTOR_INDEX])

    except KeyboardInterrupt:
        print("\nInterrupted, stopping motor.")
    finally:
        motors.stop_motor([MOTOR_INDEX])
        motors.close()
        GPIO.cleanup()  # releases the load cell's DOUT/SCK pins too;
                         # harmless if motors.close() already cleans up GPIO


if __name__ == "__main__":
    main()


# ----------------------------------------------------------------------
# Tuning notes
# ----------------------------------------------------------------------
# - Start with Ki = Kd = 0. Raise Kp until the motor visibly moves toward
#   the target force without violently overshooting/oscillating.
# - Then raise Ki slowly until steady-state error (settling short of the
#   setpoint and staying there) goes away.
# - Only add Kd if you still see overshoot/ringing you want to damp --
#   and if you do, filter read_force_kg()'s output (simple moving average
#   or exponential smoothing) first, since raw HX711 noise fed straight
#   into a derivative term produces useless, jumpy output.
# - PID_DT should roughly match how fast your load cell actually updates
#   (HX711 is ~10-80 Sa/s depending on config) -- running the PID faster
#   than your sensor just means you're reacting to stale data repeatedly.

