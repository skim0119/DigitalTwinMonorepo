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

import csv
import threading
import sys
import time
import RPi.GPIO as GPIO
import matplotlib
matplotlib.use("Agg")  # headless -- no display needed/available on the Pi
import matplotlib.pyplot as plt
from hx711 import HX711
sys.path.append('/home/mgazzola/spirob_py_code/Spirob_Code/core_nodes/motor')
from motors import Motors

# ----------------------------------------------------------------------
# Motor / pin setup (same values as xbox_controller.py)
# ----------------------------------------------------------------------
PUL = [4, 23, 22, 26]
DIR = [27, 24, 25, 16]
MOTOR_ORIENTATION = [False, False, True, True]

MOTOR_INDEX = 1          # which motor in the list you're controlling

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
CAL_FACTOR = 34820     # counts per kg (regression slope: 39.2938 counts/g)
RAW_OFFSET = 10431    # raw reading at zero load (your direct tare measurement --
                          # the regression's fitted intercept was 275748, close but
                          # this is what you actually measured with nothing on the cell)

_last_good_raw = RAW_OFFSET  # fallback so a bad HX711 read never gets silently treated as 0

# ----------------------------------------------------------------------
# Load cell is read in its own thread, NOT in the main control loop.
# get_raw_data_mean() blocks until the HX711 has fresh samples ready
# (only ~10-80 Sa/s), which is way slower than the main loop needs to spin
# to keep the motor pulsing smoothly. If that blocking call sat in the main
# loop, motors.update() would stop being called every time it ran -- the
# motor would stall for tens/hundreds of ms each PID step, then get a short
# burst of pulses, then stall again. That's what "moving so slowly" was:
# not a speed/tuning problem, the motor was being starved of pulses.
# ----------------------------------------------------------------------
_raw_lock = threading.Lock()
_latest_raw = RAW_OFFSET
_stop_reader = threading.Event()
_first_reading_ready = threading.Event()


def _load_cell_reader_loop():
    global _latest_raw, _last_good_raw
    while not _stop_reader.is_set():
        raw = load_cell.get_raw_data_mean(readings=5)
        if raw is not False:
            with _raw_lock:
                _latest_raw = raw
                _last_good_raw = raw
                _first_reading_ready.set()
        # no sleep here -- get_raw_data_mean() already blocks on the HX711's
        # own data-ready timing, so this loop naturally runs at sensor speed


def read_raw_load_cell() -> int:
    """Instantly returns the most recent reading the background thread got --
    never blocks, so the main loop can keep pulsing the motor continuously."""
    with _raw_lock:
        return _latest_raw


def read_force_kg() -> float:
    raw = read_raw_load_cell()
    return (raw - RAW_OFFSET) / CAL_FACTOR


# ----------------------------------------------------------------------
# PID setup
# ----------------------------------------------------------------------
SETPOINT_SCHEDULE = [ 1.5, 0.15] 
SETPOINT_INTERVAL_S = 30.0             # seconds to hold each setpoint before moving to the next

Kp = 300.0   # steps/sec of output per kg of error -- START HERE, see notes
Ki = 15.0
Kd = 5.0     # leave at 0 until Kp/Ki feel reasonable (load cell noise + D = bad)

PID_DT = 0.02   # trigger threshold only -- actual dt used below is measured,
                # so this doesn't need to exactly match the HX711's real rate

_integral = 0.0
_prev_error = 0.0

# ----------------------------------------------------------------------
# Run logging -- so you can see measured force vs. setpoint after the run
# ----------------------------------------------------------------------
LOG_CSV = "pid_force_log.csv"
PLOT_PNG = "pid_force_plot.png"


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
def save_log_and_plot(times, measured, setpoints, outputs):
    """Writes the run to CSV and plots measured force vs. setpoint over time."""
    if not times:
        print("No data logged -- skipping CSV/plot.")
        return

    with open(LOG_CSV, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["elapsed_s", "measured_force_kg", "setpoint_kg", "output_steps_per_s"])
        writer.writerows(zip(times, measured, setpoints, outputs))
    print(f"Saved run log to {LOG_CSV}")

    fig, ax = plt.subplots(figsize=(8, 5), dpi=150)
    ax.plot(times, measured, color="#3B6FA0", linewidth=1.5, label="measured force")
    ax.plot(times, setpoints, color="#D9534F", linestyle="--", linewidth=1.5, label="setpoint")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Force (kg)")
    ax.set_ylim(0,3.5)
    ax.set_title("PID force control: measured vs. setpoint")
    ax.legend(loc="best", frameon=False)
    ax.grid(True, linestyle="--", alpha=0.4)
    for spine in ["top", "right"]:
        ax.spines[spine].set_visible(False)
    plt.tight_layout()
    plt.savefig(PLOT_PNG)
    plt.close(fig)
    print(f"Saved plot to {PLOT_PNG}")


def main():
    reader_thread = threading.Thread(target=_load_cell_reader_loop, daemon=True)
    reader_thread.start()

    print("Waiting for first load cell reading...")
    _first_reading_ready.wait()   # don't start the PID loop on the RAW_OFFSET placeholder

    motors.start_motor([MOTOR_INDEX])
    last_pid_time = time.time()
    start_time = last_pid_time
    last_print_time = start_time          # <-- NEW

    # --- setpoint schedule state ---
    setpoint_index = 0
    current_setpoint = SETPOINT_SCHEDULE[setpoint_index]
    last_setpoint_change_time = start_time
    print(f"Setpoint {setpoint_index}: {current_setpoint} kg at t=0.0s")   # <-- add timestamp

    log_times = []
    log_measured = []
    log_setpoints = []
    log_outputs = []

    try:
        while True:
            now = time.time()

            # step to the next setpoint every SETPOINT_INTERVAL_S seconds
            if now - last_setpoint_change_time >= SETPOINT_INTERVAL_S:
                setpoint_index = (setpoint_index + 1) % len(SETPOINT_SCHEDULE)
                current_setpoint = SETPOINT_SCHEDULE[setpoint_index]
                last_setpoint_change_time = now
                print(f"Setpoint {setpoint_index}: {current_setpoint} kg at t={now - start_time:.1f}s")   # <-- add timestamp

            if now - last_pid_time >= PID_DT:
                dt = now - last_pid_time
                last_pid_time = now

                measured_force = read_force_kg()
                output = pid_step(current_setpoint, measured_force, dt)
                apply_pid_output_to_motor(output)

                log_times.append(now - start_time)
                log_measured.append(measured_force)
                log_setpoints.append(current_setpoint)
                log_outputs.append(output)

                if now - last_print_time >= 0.2:      # <-- NEW: print at most 5x/sec, not 50x/sec
                    print(f"force={measured_force:.3f}kg  setpoint={current_setpoint}kg  output={output:.1f} steps/s")
                    last_print_time = now

            motors.update([MOTOR_INDEX])

    except KeyboardInterrupt:
        print("\nInterrupted, stopping motor.")
    finally:
        motors.stop_motor([MOTOR_INDEX])
        motors.close()
        _stop_reader.set()
        reader_thread.join(timeout=1.0)  # HX711 read may be mid-sample; don't hang forever
        GPIO.cleanup()  # releases the load cell's DOUT/SCK pins too;
                         # harmless if motors.close() already cleans up GPIO
        save_log_and_plot(log_times, log_measured, log_setpoints, log_outputs)


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

