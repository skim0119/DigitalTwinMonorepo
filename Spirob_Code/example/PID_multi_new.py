import csv
import threading
import sys
import time
from collections import deque

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
PUL = [18, 10, 4, 23, 22, 26]
DIR = [14, 9, 27, 1, 25, 0]
MOTOR_ORIENTATION = [True, True, True, True, True, True]

GPIO.setmode(GPIO.BCM)
motors = Motors(
    PUL,
    DIR,
    MOTOR_ORIENTATION,
    steps_per_rev=1600,
    max_rpm=80,  # xbox script uses 30 -- try 30 here too if it still sounds rough
    pulse_width=10e-6,
    debug=False,
)

# 80 rpm -> 2133 steps/s, 30 rpm -> 800 steps/s
MAX_STEPS_PER_SEC = motors.max_rpm / 60 * motors.steps_per_rev

DOUT_PINS = [2, 5, 7, 21, 17, 12]
SCK_PINS = [3, 6, 11, 13, 20, 19]

# ----------------------------------------------------------------------
# Sensor filtering
# ----------------------------------------------------------------------
WINDOW_SIZE = 5        # rolling window of single HX711 samples (same smoothing as readings=5)
USE_MEDIAN = False     # True -> median of window (extra protection against spikes)
MAX_SAMPLE_JUMP_KG = 0.3     # reject a single sample this far from the recent median
MAX_CONSECUTIVE_REJECTS = 4  # ...unless this many in a row agree (real fast change)

# ----------------------------------------------------------------------
# Control loop
# ----------------------------------------------------------------------
# The PID now runs once per NEW load-cell sample (event driven), so dt is
# always the real time between sensor updates (~12.5 ms at 80 SPS).
# The main loop just polls for new samples at this interval.
MAIN_LOOP_POLL_S = 0.002
MAX_DT = 0.1                 # clamp dt if a sensor stalls, so I/D don't blow up

ERROR_DEADBAND_KG = 0.02     # |error| below this -> stop (no chatter around setpoint)
MAX_ACCEL = 600.0           # steps/s^2 -- max rate of change of commanded speed
MIN_ACTIVE_STEPS_PER_SEC = 20.0  # below this, don't pulse at all

PULSE_IDLE_POLL_S = 0.005
STATUS_PRINT_INTERVAL_S = 0.2


class MotorChannel:
    """Everything one motor's force-PID loop needs: its own load cell +
    reader thread, its own pulse thread, calibration, PID state, and
    setpoint schedule."""

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

        # --- reader-thread state ---
        self._raw_lock = threading.Lock()
        self._window = deque(maxlen=WINDOW_SIZE)
        self._latest_raw = raw_offset
        self._sample_count = 0           # increments on every good sample
        self._failed_reads = 0           # HX711 read failed (timing/power-down)
        self._rejected_reads = 0         # read OK but rejected as a glitch
        self._stop_reader = threading.Event()
        self.first_reading_ready = threading.Event()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)

        # --- pulse-thread state ---
        self.should_pulse = False
        self.last_output = 0.0
        self._stop_pulse = threading.Event()
        self._pulse_thread = threading.Thread(target=self._pulse_loop, daemon=True)

        # --- PID state ---
        self._integral = 0.0
        self._prev_meas = None           # derivative on measurement
        self._cmd = 0.0                  # slew-limited commanded speed (steps/s)
        self._current_dir = None         # avoid re-setting DIR pin every cycle
        self._last_seen_sample = 0
        self._last_step_time = None

        # --- setpoint schedule state ---
        self.setpoint_index = 0
        self.current_setpoint = setpoint_schedule[0]
        self._last_setpoint_change_time = None  # set in start()

        # --- stats ---
        self._reader_start_time = None

        # --- per-motor logs ---
        self.log_times = []
        self.log_measured = []
        self.log_setpoints = []
        self.log_outputs = []

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    def start(self, start_time):
        self._last_setpoint_change_time = start_time
        self._reader_start_time = time.time()
        self._reader_thread.start()
        print(f"[{self.name}] Waiting for first load cell reading...")
        self.first_reading_ready.wait()
        print(f"[{self.name}] Setpoint {self.setpoint_index}: {self.current_setpoint} kg at t=0.0s")
        motors.start_motor([self.motor_index])
        self._pulse_thread.start()

    def stop(self):
        """Stops this motor and both of its background threads."""
        self.should_pulse = False
        motors.stop_motor([self.motor_index])
        self._stop_pulse.set()
        self._pulse_thread.join(timeout=1.0)
        self._stop_reader.set()
        self._reader_thread.join(timeout=1.0)

    def sample_rate_hz(self):
        if self._reader_start_time is None:
            return 0.0
        elapsed = time.time() - self._reader_start_time
        with self._raw_lock:
            n = self._sample_count
        return n / elapsed if elapsed > 0 else 0.0

    # ------------------------------------------------------------------
    # background threads
    # ------------------------------------------------------------------
    def _reader_loop(self):
        """Reads ONE sample per call and keeps a rolling window, so a new
        filtered value is available on every HX711 sample (~80 Hz) instead
        of every 5th sample (~16 Hz) like get_raw_data_mean(readings=5)."""
        # NOTE: get_raw_data_mean(readings=1) skips the library's outlier
        # filter AND turns a failed read into 0 (not False) -- that 0 was
        # causing huge force spikes. So read single samples with _read(),
        # which really returns False on failure, and filter glitches here.
        consecutive_rejects = 0
        while not self._stop_reader.is_set():
            raw = self.load_cell._read()
            if raw is False or raw is None:
                self._failed_reads += 1
                continue

            # glitch rejection: a single sample far from the recent median is
            # almost certainly a corrupted bit-bang read, not real force
            if len(self._window) >= 3:
                med = sorted(self._window)[len(self._window) // 2]
                if abs(raw - med) / abs(self.cal_factor) > MAX_SAMPLE_JUMP_KG:
                    consecutive_rejects += 1
                    self._rejected_reads += 1
                    if consecutive_rejects < MAX_CONSECUTIVE_REJECTS:
                        continue
                    # several in a row agree -> it's a real change, accept it
                    self._window.clear()
            consecutive_rejects = 0

            with self._raw_lock:
                self._window.append(raw)
                if USE_MEDIAN:
                    s = sorted(self._window)
                    self._latest_raw = s[len(s) // 2]
                else:
                    self._latest_raw = sum(self._window) / len(self._window)
                self._sample_count += 1
                self.first_reading_ready.set()
            # no sleep -- get_raw_data_mean() blocks on the HX711's data-ready

    def _pulse_loop(self):
        while not self._stop_pulse.is_set():
            if self.should_pulse:
                motors.update([self.motor_index])
            else:
                time.sleep(PULSE_IDLE_POLL_S)

    # ------------------------------------------------------------------
    # control
    # ------------------------------------------------------------------
    def _read_force_kg(self):
        with self._raw_lock:
            raw = self._latest_raw
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

        # derivative on measurement (no kick when the setpoint changes)
        if self._prev_meas is None:
            derivative = 0.0
        else:
            derivative = -(measured - self._prev_meas) / dt
        self._prev_meas = measured

        if abs(error) < ERROR_DEADBAND_KG:
            # close enough: ramp to a stop, don't integrate
            target = 0.0
        else:
            self._integral += error * dt
            target = self.kp * error + self.ki * self._integral + self.kd * derivative

            # clamp + anti-windup
            if target > MAX_STEPS_PER_SEC:
                self._integral -= error * dt
                target = MAX_STEPS_PER_SEC
            elif target < -MAX_STEPS_PER_SEC:
                self._integral -= error * dt
                target = -MAX_STEPS_PER_SEC

        # slew-rate limit: speed ramps smoothly and reversals pass through 0
        max_delta = MAX_ACCEL * dt
        self._cmd = max(self._cmd - max_delta, min(self._cmd + max_delta, target))
        return self._cmd  # steps/sec, sign = direction

    def _apply_output(self, output_steps_per_sec):
        if abs(output_steps_per_sec) < MIN_ACTIVE_STEPS_PER_SEC:
            self.should_pulse = False
            return

        new_dir = output_steps_per_sec >= 0
        if new_dir != self._current_dir:
            if new_dir:
                motors.dir_motor_forward([self.motor_index])
            else:
                motors.dir_motor_backward([self.motor_index])
            self._current_dir = new_dir

        speed_fraction = min(abs(output_steps_per_sec) / MAX_STEPS_PER_SEC, 1.0)
        motors.set_motor_speed([self.motor_index], speed_fraction)
        self.should_pulse = True

    def step(self, now, start_time):
        """Runs one PID update ONLY if a new load-cell sample has arrived
        since the last call. Returns True if it ran, False otherwise."""
        self._maybe_advance_setpoint(now, start_time)

        with self._raw_lock:
            n = self._sample_count
        if n == self._last_seen_sample:
            return False  # no new data -- keep current command
        self._last_seen_sample = n

        if self._last_step_time is None:
            dt = 1.0 / 80.0
        else:
            dt = now - self._last_step_time
        dt = min(max(dt, 1e-3), MAX_DT)
        self._last_step_time = now

        measured_force = self._read_force_kg()
        output = self._pid_step(measured_force, dt)
        self.last_output = output
        self._apply_output(output)

        self.log_times.append(now - start_time)
        self.log_measured.append(measured_force)
        self.log_setpoints.append(self.current_setpoint)
        self.log_outputs.append(output)
        return True


# ----------------------------------------------------------------------
# Per-motor configuration
# NOTE: with a stepper (speed-commanded actuator), P alone already drives
# steady-state force error to zero -- keep ki small; it mostly adds overshoot.
# Gains were tuned at a different loop rate, so expect to re-tune.
# ----------------------------------------------------------------------
channel0 = MotorChannel(
    name="motor0", motor_index=0,
    cal_factor=34170, raw_offset=170539,
    kp=550.0, ki=10.0, kd=5.0,
    setpoint_schedule=[0.1], setpoint_interval_s=300,
)

channel1 = MotorChannel(
    name="motor1", motor_index=1,
    cal_factor=34170, raw_offset=-285116.4,
    kp=550.0, ki=10.0, kd=5.0,
    setpoint_schedule=[0.1], setpoint_interval_s=300,
)

channel2 = MotorChannel(
    name="motor2", motor_index=2,
    cal_factor=39293.8, raw_offset=276320.0,
    kp=550.0, ki=10.0, kd=5.0,
    setpoint_schedule=[0.1], setpoint_interval_s=300.0,
)

channel3 = MotorChannel(
    name="motor3", motor_index=3,
    cal_factor=34824.0, raw_offset=10569.0,
    kp=550.0, ki=10.0, kd=5.0,
    setpoint_schedule=[0.1], setpoint_interval_s=300.0,
)

channel4 = MotorChannel(
    name="motor4", motor_index=4,
    cal_factor=33760.0, raw_offset=236592.0,
    kp=750.0, ki=6.0, kd=5.0,
    setpoint_schedule=[0.1], setpoint_interval_s=300.0,
)

channel5 = MotorChannel(
    name="motor5", motor_index=5,
    cal_factor=31950.0, raw_offset=-170491,
    kp=150.0, ki=10.0, kd=5.0,
    setpoint_schedule=[0.1,0.7,1.3,1.5,2, 2.5,2,1.5,1.3,0.7], setpoint_interval_s=5,
)

CHANNELS = [channel0, channel1, channel2, channel3, channel4, channel5]

LOG_CSV_TEMPLATE = "pid_force_log_{name}.csv"
COMBINED_PLOT_PNG = "pid_force_plot_all_motors.png"
CHANNEL_COLORS = ["#3B6FA0", "#D9534F", "#5CB85C", "#F0AD4E", "#9B59B6", "#1ABC9C"]


def save_log_csv(channel):
    if not channel.log_times:
        print(f"[{channel.name}] No data logged -- skipping CSV.")
        return
    csv_path = LOG_CSV_TEMPLATE.format(name=channel.name)
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["elapsed_s", "measured_force_kg", "setpoint_kg", "output_steps_per_s"])
        writer.writerows(zip(channel.log_times, channel.log_measured,
                             channel.log_setpoints, channel.log_outputs))
    print(f"[{channel.name}] Saved run log to {csv_path}")


def save_combined_plot(channels):
    channels_with_data = [ch for ch in channels if ch.log_times]
    if not channels_with_data:
        print("No data logged for any channel -- skipping combined plot.")
        return

    fig, ax = plt.subplots(figsize=(10, 6), dpi=150)
    for i, ch in enumerate(channels_with_data):
        color = CHANNEL_COLORS[i % len(CHANNEL_COLORS)]
        ax.plot(ch.log_times, ch.log_measured, color=color, linewidth=1.5,
                label=f"{ch.name} force")
        ax.plot(ch.log_times, ch.log_setpoints, color=color, linestyle="--",
                linewidth=1.2, alpha=0.6, label=f"{ch.name} setpoint")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Force (kg)")
    ax.set_ylim(-0.5, 3)
    ax.set_title("PID force control: all motors, measured vs. setpoint")
    ax.legend(loc="best", frameon=False, ncol=2, fontsize=8)
    ax.grid(True, linestyle="--", alpha=0.4)
    for spine in ["top", "right"]:
        ax.spines[spine].set_visible(False)
    plt.tight_layout()
    plt.savefig(COMBINED_PLOT_PNG)
    plt.close(fig)
    print(f"Saved combined plot to {COMBINED_PLOT_PNG}")


def print_status(channels):
    parts = []
    for ch in channels:
        if not ch.log_measured:
            parts.append(f"{ch.name}: (no data yet)")
            continue
        parts.append(
            f"{ch.name}: F={ch.log_measured[-1]:.3f} SP={ch.current_setpoint} "
            f"out={ch.log_outputs[-1]:.0f} {ch.sample_rate_hz():.0f}Hz"
            + ("" if ch.should_pulse else " (idle)")
        )
    print("  |  ".join(parts))


def main():
    start_time = time.time()
    for ch in CHANNELS:
        ch.start(start_time)  # starts reader thread, motor, and pulse thread

    last_print_time = start_time

    try:
        while True:
            now = time.time()
            for ch in CHANNELS:
                ch.step(now, start_time)  # no-op unless that channel has a new sample

            if now - last_print_time >= STATUS_PRINT_INTERVAL_S:
                print_status(CHANNELS)
                last_print_time = now

            time.sleep(MAIN_LOOP_POLL_S)

    except KeyboardInterrupt:
        print("\nInterrupted, stopping.")
    finally:
        print("\nMeasured load-cell sample rates:")
        for ch in CHANNELS:
            print(f"  {ch.name}: {ch.sample_rate_hz():.1f} Hz good, "
                  f"{ch._failed_reads} failed reads, {ch._rejected_reads} glitches rejected")
        for ch in CHANNELS:
            ch.stop()
        motors.close()
        GPIO.cleanup()
        for ch in CHANNELS:
            save_log_csv(ch)
        save_combined_plot(CHANNELS)


if __name__ == "__main__":
    main()


# ----------------------------------------------------------------------
# Tuning notes
# ----------------------------------------------------------------------
# - Check the printed sample rates first. If they're ~10 Hz, the HX711 RATE
#   pin isn't set for 80 SPS; if they're well under 80 with all 6 running,
#   the Pi/GIL is the bottleneck.
# - Start with Ki = Kd = 0, raise Kp until it reaches the target briskly
#   without oscillating. Then add a little Ki only if needed.
# - Still rough? Lower MAX_ACCEL (smoother ramps) and/or max_rpm.
# - Spiky force trace? Set USE_MEDIAN = True.
# - Hunting around setpoint? Widen ERROR_DEADBAND_KG slightly.
# - Requires the patched motors.py (per-motor pulse state).
