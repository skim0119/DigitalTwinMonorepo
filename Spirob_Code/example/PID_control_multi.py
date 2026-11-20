"""
Force-control loop for TWO stepper motors, each running its own independent
PID loop against its own load cell -- motor0 and motor1 -- in the same
script and the same main loop.

This mirrors motor0_PID.py exactly (same reader-thread pattern, same plain
PID step with no deadband/clamp/ramp/filtering, same setpoint-schedule
logic, same throttled printing) -- it's just been split into a MotorChannel
class so that state doesn't get duplicated by hand for a second motor.

WHY A CLASS: motor0_PID.py had module-level globals for PID state
(_integral, _prev_error), its own load-cell reader thread + lock, and its
own setpoint-schedule state. Duplicating all of that a second time for
motor1 as more globals means two nearly-identical sets of variables and a
real risk of a copy/paste bug where a function accidentally reads/writes
the wrong motor's state. MotorChannel bundles all of that per-motor state
into one object -- create one instance per motor, and the main loop calls
`channel.step(...)` on each of them every cycle.

WHAT STAYS SHARED: the `motors` object (one Motors instance driving all
pins) and MAX_STEPS_PER_SEC, since those describe the shared hardware
driver, not any one motor's control loop.
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
# Motor / pin setup (shared across all motors)
# ----------------------------------------------------------------------
PUL = [4, 23, 22, 26]
DIR = [27, 24, 25, 16]
MOTOR_ORIENTATION = [False, False, True, True]

GPIO.setmode(GPIO.BCM)

motors = Motors(
    PUL,
    DIR,
    MOTOR_ORIENTATION,
    steps_per_rev=1600,
    max_rpm=30,
    pulse_width=10e-6,
    debug=False,
)

MAX_STEPS_PER_SEC = motors.max_rpm / 60 * motors.steps_per_rev

DOUT_PINS = [5, 21, 13, 19]
SCK_PINS = [6, 20, 17, 12]

PID_DT = 0.02  # trigger threshold -- actual dt used is measured per-cycle


class MotorChannel:
    """Everything one motor's force-PID loop needs: its own load cell +
    reader thread, calibration, PID state, and setpoint schedule."""

    def __init__(self, name, motor_index, cal_factor, raw_offset,
                 kp, ki, kd, setpoint_schedule, setpoint_interval_s):
        self.name = name
        self.motor_index = motor_index
        self.cal_factor = cal_factor
        self.raw_offset = raw_offset
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint_schedule = setpoint_schedule
        self.setpoint_interval_s = setpoint_interval_s

        self.load_cell = HX711(
            dout_pin=DOUT_PINS[motor_index],
            pd_sck_pin=SCK_PINS[motor_index],
            select_channel='A',
            gain_channel_A=64,
        )
        self.load_cell.reset()

        # --- reader-thread state (same pattern as motor0_PID.py) ---
        self._raw_lock = threading.Lock()
        self._latest_raw = raw_offset
        self._last_good_raw = raw_offset
        self._stop_reader = threading.Event()
        self.first_reading_ready = threading.Event()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)

        # --- PID state ---
        self._integral = 0.0
        self._prev_error = 0.0

        # --- setpoint schedule state ---
        self.setpoint_index = 0
        self.current_setpoint = setpoint_schedule[0]
        self._last_setpoint_change_time = None  # set in start()

        # --- per-motor logs (kept separate so each gets its own plot) ---
        self.log_times = []
        self.log_measured = []
        self.log_setpoints = []
        self.log_outputs = []

    def start(self, start_time):
        self._last_setpoint_change_time = start_time
        self._reader_thread.start()
        print(f"[{self.name}] Waiting for first load cell reading...")
        self.first_reading_ready.wait()  # don't start the PID loop on the raw_offset placeholder
        print(f"[{self.name}] Setpoint {self.setpoint_index}: {self.current_setpoint} kg at t=0.0s")

    def _reader_loop(self):
        while not self._stop_reader.is_set():
            raw = self.load_cell.get_raw_data_mean(readings=5)
            if raw is not False:
                with self._raw_lock:
                    self._latest_raw = raw
                    self._last_good_raw = raw
                    self.first_reading_ready.set()
            # no sleep here -- get_raw_data_mean() already blocks on the HX711's
            # own data-ready timing, so this loop naturally runs at sensor speed

    def _read_raw(self):
        with self._raw_lock:
            return self._latest_raw

    def _read_force_kg(self):
        raw = self._read_raw()
        return (raw - self.raw_offset) / self.cal_factor

    def _maybe_advance_setpoint(self, now, start_time):
        if now - self._last_setpoint_change_time >= self.setpoint_interval_s:
            self.setpoint_index = (self.setpoint_index + 1) % len(self.setpoint_schedule)
            self.current_setpoint = self.setpoint_schedule[self.setpoint_index]
            self._last_setpoint_change_time = now
            print(f"[{self.name}] Setpoint {self.setpoint_index}: "
                  f"{self.current_setpoint} kg at t={now - start_time:.1f}s")

    def _pid_step(self, measured, dt):
        error = self.current_setpoint - measured

        self._integral += error * dt
        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative

        # clamp to actuator limits + basic anti-windup (back the integral off
        # by the amount that pushed us past the limit)
        if output > MAX_STEPS_PER_SEC:
            self._integral -= error * dt
            output = MAX_STEPS_PER_SEC
        elif output < -MAX_STEPS_PER_SEC:
            self._integral -= error * dt
            output = -MAX_STEPS_PER_SEC

        return output  # steps/sec, sign = direction

    def _apply_output(self, output_steps_per_sec):
        """This is the piece that answers 'how do I use the PID output on the motor'."""
        if output_steps_per_sec >= 0:
            motors.dir_motor_forward([self.motor_index])
        else:
            motors.dir_motor_backward([self.motor_index])

        speed_fraction = min(abs(output_steps_per_sec) / MAX_STEPS_PER_SEC, 1.0)
        motors.set_motor_speed([self.motor_index], speed_fraction)

    def step(self, now, dt, start_time):
        """One PID update for this motor: advance its schedule, read its
        load cell, compute + apply its output, and log the sample."""
        self._maybe_advance_setpoint(now, start_time)

        measured_force = self._read_force_kg()
        output = self._pid_step(measured_force, dt)
        self._apply_output(output)

        self.log_times.append(now - start_time)
        self.log_measured.append(measured_force)
        self.log_setpoints.append(self.current_setpoint)
        self.log_outputs.append(output)

        return measured_force, output

    def stop_reader(self):
        self._stop_reader.set()
        self._reader_thread.join(timeout=1.0)


# ----------------------------------------------------------------------
# Configure each motor's channel -- calibration, gains, and setpoint
# schedule are all per-motor. EDIT THESE FOR YOUR TWO LOAD CELLS/MOTORS.
# ----------------------------------------------------------------------
channel0 = MotorChannel(
    name="motor0",
    motor_index=0,
    cal_factor=39293.8,      # counts per kg -- motor0's 5-point calibration
    raw_offset=276320.0,     # motor0's directly measured 0g reading
    kp=300.0, ki=15.0, kd=5.0,
    setpoint_schedule=[0.2, 1.0],
    setpoint_interval_s=30.0,
)

channel1 = MotorChannel(
    name="motor1",
    motor_index=1,
    # TODO: fill in motor1's actual calibration/gains/schedule
    cal_factor=34824.0,
    raw_offset=10569.0,
    kp=300.0, ki=15.0, kd=5.0,
    setpoint_schedule=[0.3, 1.0],
    setpoint_interval_s=40.0,
)

CHANNELS = [channel0, channel1]
#CHANNELS = [channel0]
LOG_CSV_TEMPLATE = "pid_force_log_{name}.csv"
PLOT_PNG_TEMPLATE = "pid_force_plot_{name}.png"


def save_log_and_plot(channel):
    """Writes the run to CSV and plots measured force vs. setpoint over time."""
    if not channel.log_times:
        print(f"[{channel.name}] No data logged -- skipping CSV/plot.")
        return

    csv_path = LOG_CSV_TEMPLATE.format(name=channel.name)
    png_path = PLOT_PNG_TEMPLATE.format(name=channel.name)

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["elapsed_s", "measured_force_kg", "setpoint_kg", "output_steps_per_s"])
        writer.writerows(zip(channel.log_times, channel.log_measured,
                              channel.log_setpoints, channel.log_outputs))
    print(f"[{channel.name}] Saved run log to {csv_path}")

    fig, ax = plt.subplots(figsize=(8, 5), dpi=150)
    ax.plot(channel.log_times, channel.log_measured, color="#3B6FA0", linewidth=1.5, label="measured force")
    ax.plot(channel.log_times, channel.log_setpoints, color="#D9534F", linestyle="--", linewidth=1.5, label="setpoint")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Force (kg)")
    ax.set_ylim(0, 3.5)
    ax.set_title(f"PID force control ({channel.name}): measured vs. setpoint")
    ax.legend(loc="best", frameon=False)
    ax.grid(True, linestyle="--", alpha=0.4)
    for spine in ["top", "right"]:
        ax.spines[spine].set_visible(False)
    plt.tight_layout()
    plt.savefig(png_path)
    plt.close(fig)
    print(f"[{channel.name}] Saved plot to {png_path}")


def main():
    start_time = time.time()
    for ch in CHANNELS:
        ch.start(start_time)

    motor_indices = [ch.motor_index for ch in CHANNELS]
    for idx in motor_indices:                 # <-- was: motors.start_motor(motor_indices)
        motors.start_motor([idx])
    #motors.start_motor(motor_indices)

    last_pid_time = time.time()
    last_print_time = start_time

    try:
        while True:
            now = time.time()

            if now - last_pid_time >= PID_DT:
                dt = now - last_pid_time
                last_pid_time = now

                results = [(ch.name, *ch.step(now, dt, start_time), ch.current_setpoint) for ch in CHANNELS]

                if now - last_print_time >= 0.2:      # print at most 5x/sec, not 50x/sec
                    line = "  |  ".join(
                        f"{name}: force={f:.3f}kg setpoint={sp}kg output={o:.1f} steps/s"
                        for name, f, o, sp in results
                    )
                    print(line)
                    last_print_time = now

            # pulse ALL motors together every iteration -- this must keep
            # running as fast as possible for smooth motion
            for idx in motor_indices:          # <-- was: motors.update(motor_indices)
               motors.update([idx])
            #motors.update(motor_indices)

    except KeyboardInterrupt:
        print("\nInterrupted, stopping motors.")
    finally:
        for idx in motor_indices:              # <-- was: motors.stop_motor(motor_indices)
            motors.stop_motor([idx])
        #motors.stop_motor(motor_indices)
        motors.close()
        for ch in CHANNELS:
            ch.stop_reader()
        GPIO.cleanup()  # releases the load cells' DOUT/SCK pins too;
                         # harmless if motors.close() already cleans up GPIO
        for ch in CHANNELS:
            save_log_and_plot(ch)


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
# - Tune each motor's Kp/Ki/Kd independently -- there's no reason to expect
#   the same gains work for both motors/load cells/mechanical setups.

