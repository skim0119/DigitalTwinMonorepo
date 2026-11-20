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
from motors_new import Motors

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

# motors.update() appears to block for a duration that grows as commanded
# speed shrinks (measured 0.075s-0.9s+ as output dropped from ~2.3 to
# ~0.5 steps/s, and it hangs outright near 0) -- almost certainly a
# blocking sleep-per-step inside motors.py scaled by 1/speed. Below this
# floor we just don't drive/pulse the motor that cycle rather than risk
# a multi-second (or infinite) stall of the whole loop. Tune by watching
# for "!! motors.update() took" warnings -- raise this until they stop.
MIN_ACTIVE_STEPS_PER_SEC = 10.0

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
        """This is the piece that answers 'how do I use the PID output on the motor'.

        Returns True if the motor should actually be pulsed this cycle
        (via motors.update()), False if the commanded speed is too low to
        safely do that -- see MIN_ACTIVE_STEPS_PER_SEC below for why."""
        if abs(output_steps_per_sec) < MIN_ACTIVE_STEPS_PER_SEC:
            # Close enough to setpoint that the PID wants to barely creep.
            # motors.update() appears to block for a duration tied to the
            # commanded speed (near-zero speed -> very long/near-infinite
            # block), so we deliberately skip driving the motor at all
            # rather than ask for a speed that could hang the whole loop.
            return False

        if output_steps_per_sec >= 0:
            motors.dir_motor_forward([self.motor_index])
        else:
            motors.dir_motor_backward([self.motor_index])

        speed_fraction = min(abs(output_steps_per_sec) / MAX_STEPS_PER_SEC, 1.0)
        motors.set_motor_speed([self.motor_index], speed_fraction)
        return True

    def step(self, now, dt, start_time, advance_schedule=True):
        """One PID update for this motor: advance its schedule (unless
        advance_schedule=False, which holds at the current setpoint
        instead -- used for the gap before switching to the next motor),
        read its load cell, compute + apply its output, and log the sample.

        Returns (measured_force, output, should_pulse) -- should_pulse
        tells the caller whether it's safe to call motors.update() for
        this motor this cycle (see _apply_output)."""
        if advance_schedule:
            self._maybe_advance_setpoint(now, start_time)

        measured_force = self._read_force_kg()
        output = self._pid_step(measured_force, dt)
        should_pulse = self._apply_output(output)

        self.log_times.append(now - start_time)
        self.log_measured.append(measured_force)
        self.log_setpoints.append(self.current_setpoint)
        self.log_outputs.append(output)

        return measured_force, output, should_pulse

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
    setpoint_schedule=[0.3, 1.0],
    setpoint_interval_s=15.0,
)

channel1 = MotorChannel(
    name="motor1",
    motor_index=1,
    # TODO: fill in motor1's actual calibration/gains/schedule
    cal_factor=34824.0,
    raw_offset=10569.0,
    kp=300.0, ki=15.0, kd=5.0,
    setpoint_schedule=[0.3, 1.0],
    setpoint_interval_s=15.0,
)

CHANNELS = [channel0, channel1]

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



# How long to let each motor run, by itself, before moving on to the next
# one. Default = one full pass through a 2-point schedule at 30s/setpoint.
# Only ONE channel's reader thread + motor is ever active at a time --
# this is for isolating whether running them concurrently is what's
# breaking motion, vs. something in the per-motor logic itself.
RUN_DURATION_PER_CHANNEL_S = 30.0

# Gap between one motor finishing its setpoint schedule and the next motor
# starting. Unlike a plain pause, the finishing motor keeps its PID loop
# running and actively holding at its last setpoint during this gap --
# it doesn't idle/stop until the moment the next motor is about to start.
SWITCH_DELAY_S = 5.0


def run_channel_standalone(ch, run_duration_s, hold_duration_s=0.0):
    """Runs one motor's full PID force-control loop by itself: first for
    run_duration_s cycling through its setpoint schedule, then (if
    hold_duration_s > 0) for an additional hold_duration_s with the PID
    loop still running but holding at whatever setpoint it ended on --
    that's the gap before the next motor starts. Only stops the motor at
    the very end of both phases."""
    print(f"\n=== Starting {ch.name} (running alone for {run_duration_s:.0f}s) ===")
    channel_start = time.time()
    ch.start(channel_start)
    motors.start_motor([ch.motor_index])

    last_pid_time = time.time()
    last_print_time = channel_start
    total_duration_s = run_duration_s + hold_duration_s
    should_pulse = False  # in case motors.update() would be reached before the first ch.step()

    try:
        while time.time() - channel_start < total_duration_s:
            now = time.time()
            advancing = (now - channel_start) < run_duration_s

            if now - last_pid_time >= PID_DT:
                dt = now - last_pid_time
                last_pid_time = now

                measured, output, should_pulse = ch.step(now, dt, channel_start, advance_schedule=advancing)

                if now - last_print_time >= 0.2:  # print at most 5x/sec, not 50x/sec
                    phase = "" if advancing else " (holding, next motor incoming)"
                    note = "" if should_pulse else " (below min speed, not pulsing)"
                    print(f"{ch.name}{phase}: force={measured:.3f}kg "
                          f"setpoint={ch.current_setpoint}kg output={output:.1f} steps/s{note}")
                    last_print_time = now

            # pulse this motor every iteration -- this must keep running as
            # fast as possible for smooth motion. Skipped entirely when the
            # commanded speed is below MIN_ACTIVE_STEPS_PER_SEC, since
            # motors.update() can block for a long time (or hang) at very
            # low speeds -- see MIN_ACTIVE_STEPS_PER_SEC's comment.
            if should_pulse:
                _update_start = time.time()
                motors.update([ch.motor_index])
                _update_elapsed = time.time() - _update_start
                if _update_elapsed > 0.05:
                    print(f"[{ch.name}] !! motors.update() took {_update_elapsed:.3f}s "
                          f"(output was {output:.1f} steps/s)")

    finally:
        motors.stop_motor([ch.motor_index])
        ch.stop_reader()
        save_log_and_plot(ch)
        print(f"=== Done with {ch.name} ===")


def main():
    try:
        for i, ch in enumerate(CHANNELS):
            is_last = (i == len(CHANNELS) - 1)
            hold = 0.0 if is_last else SWITCH_DELAY_S
            run_channel_standalone(ch, RUN_DURATION_PER_CHANNEL_S, hold_duration_s=hold)
    except KeyboardInterrupt:
        print("\nInterrupted, stopping.")
    finally:
        motors.close()
        GPIO.cleanup()  # releases the load cells' DOUT/SCK pins too;
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
# - Tune each motor's Kp/Ki/Kd independently -- there's no reason to expect
#   the same gains work for both motors/load cells/mechanical setups.

