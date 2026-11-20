import argparse


def parse_args():
    parser = argparse.ArgumentParser(description="Run load-cell PID control.")
    parser.add_argument("--motor", type=int, choices=range(6),
                        help="Run only this motor and its HX711 (default: all motors).")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()


import csv
import json
import math
import os
import socket
import threading
import sys
import time
from pathlib import Path
import RPi.GPIO as GPIO
import matplotlib
matplotlib.use("Agg")  # headless -- no display needed/available on the Pi
import matplotlib.pyplot as plt
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "core_nodes/motor"))
from motors import Motors

# ----------------------------------------------------------------------
# Motor / pin setup (shared across all motors)
# ----------------------------------------------------------------------
def nonnegative_setting(value):
    if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
        raise ValueError("PID gains and deadband in pins.json must be finite nonnegative numbers")
    return value


with (Path(__file__).resolve().parents[2] / "pins.json").open() as config_file:
    settings = json.load(config_file)
motor_config = settings["motors"]
if [motor["motor"] for motor in motor_config] != list(range(6)):
    raise ValueError("pins.json must list motors 0 through 5 in order")

FORCE_DEADBAND_G = nonnegative_setting(settings["pid_deadband_g"])
CHANNEL_CONFIGS = tuple(
    dict(name=f"motor{index}", motor_index=index, setpoint_schedule=[0.1],
         setpoint_interval_s=300, **{
             gain: nonnegative_setting(motor["pid"][gain]) for gain in ("kp", "ki", "kd")
         })
    for index, motor in enumerate(motor_config)
)
PUL = [18, 10, 4, 23, 22, 26]
DIR = [14, 9, 27, 1, 25, 0]
MOTOR_ORIENTATION = [True, True, True, True, True, True]

GPIO.setmode(GPIO.BCM)
motors = Motors(
    PUL,
    DIR,
    MOTOR_ORIENTATION,
    steps_per_rev=1600,
    max_rpm=80, # 30
    pulse_width=10e-6,
    debug=False,
)

# 800
MAX_STEPS_PER_SEC = motors.max_rpm / 60 * motors.steps_per_rev

MIN_ACTIVE_STEPS_PER_SEC = 0.3

# shortcut: driver timing is unverified, check the installed driver's DIR spec before motor testing.
DIR_GUARD_S = 10e-6

PID_DT = 0.065
MAX_READING_AGE_NS = 1_000_000_000  # Five conversions can take 0.5 s at 10 SPS.
MONITOR_ADDRESS = ("127.0.0.1", int(os.getenv("SPIROB_MONITOR_PORT", "5050")))


def fetch_loads(stream, channels):
    try:
        stream.send(b"hx711")
        rows = stream.recv(512).splitlines()
        if len(rows) != 6:
            raise ValueError("expected six load-cell slots")
        now = time.monotonic_ns()
        loads = []
        for channel in channels:
            timestamp, force = rows[channel.motor_index].split()
            timestamp, force = int(timestamp), float(force)
            if timestamp == 0 or not 0 <= now - timestamp < MAX_READING_AGE_NS or not math.isfinite(force):
                raise ValueError(f"{channel.name} has no fresh load reading")
            loads.append((force, timestamp + MAX_READING_AGE_NS))
        return loads
    except (OSError, ValueError) as error:
        raise RuntimeError(f"HX711 streamer {MONITOR_ADDRESS}: {error}") from error


class TrialLimitReached(Exception):
    pass


class MotorChannel:
    """Force PID state, pulse thread, and setpoint schedule for one motor."""

    def __init__(self, name, motor_index, kp, ki, kd, setpoint_schedule, setpoint_interval_s,
                 *, deadband_g=FORCE_DEADBAND_G, max_tension_kg=None, max_tightening_steps=None):
        self.name = name
        self.motor_index = motor_index
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint_schedule = setpoint_schedule
        self.setpoint_interval_s = setpoint_interval_s
        self.deadband_kg = deadband_g / 1000
        self.max_tension_kg = max_tension_kg
        self.max_tightening_steps = max_tightening_steps
        self.tightening_steps = 0

        # --- pulse-thread state -- this is what keeps this motor's pulse
        # timing independent of the other motor's ---
        self._pulse_command = (0.0, 0)
        self._speed_changed = threading.Event()
        self._stop_pulse = threading.Event()
        self._pulse_thread = threading.Thread(target=self._pulse_loop, daemon=True)

        # --- PID state ---
        self._integral = 0.0
        self._prev_error = 0.0
        self._active_direction = 0

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
        print(f"[{self.name}] Setpoint {self.setpoint_index}: {self.current_setpoint} kg at t=0.0s")
        motors.start_motor([self.motor_index])
        self._pulse_thread.start()

    def _pulse_loop(self):
        index = self.motor_index
        forward = None
        last_step = float("-inf")
        while not self._stop_pulse.is_set():
            self._speed_changed.clear()
            if self._stop_pulse.is_set():
                break
            speed, deadline = self._pulse_command
            if speed == 0.0 or time.monotonic_ns() >= deadline:
                self._speed_changed.wait()
                continue

            until_step = last_step + 1 / abs(speed) - time.monotonic()
            if until_step > 0:
                self._speed_changed.wait(until_step)
                continue

            if (speed > 0) != forward:
                time.sleep(DIR_GUARD_S)
                forward = speed > 0
                if forward:
                    motors.dir_motor_forward(index)
                else:
                    motors.dir_motor_backward(index)
                time.sleep(DIR_GUARD_S)

            if (self._stop_pulse.is_set() or self._speed_changed.is_set()
                    or time.monotonic_ns() >= deadline):
                continue
            motors.set_motor_speed(index, abs(speed) / MAX_STEPS_PER_SEC)
            last_step = time.monotonic()
            edge = motors.update(index, wait=False)
            time.sleep(motors.pulse_width)
            motors.update(index, wait=False)
            if forward and edge == 1:
                self.tightening_steps += 1
                if self.tightening_steps == self.max_tightening_steps:
                    self._stop_pulse.set()
                    return

            time.sleep(motors.pulse_width)

    def _maybe_advance_setpoint(self, now, start_time):
        if now - self._last_setpoint_change_time >= self.setpoint_interval_s:
            self.setpoint_index = (self.setpoint_index + 1) % len(self.setpoint_schedule)
            self.current_setpoint = self.setpoint_schedule[self.setpoint_index]
            self._last_setpoint_change_time = now
            print(f"[{self.name}] Setpoint {self.setpoint_index}: "
                  f"{self.current_setpoint} kg at t={now - start_time:.1f}s")

    def _pid_step(self, measured, dt):
        measured = max(0.0, measured)
        error = self.current_setpoint - measured

        # Stop closer to target than the threshold for restarting.
        if self._active_direction and self._active_direction * error <= self.deadband_kg / 2:
            self._active_direction = 0
            self._integral = 0.0
            return 0.0
        if not self._active_direction:
            if abs(error) <= self.deadband_kg:
                return 0.0
            self._active_direction = 1 if error > 0 else -1
            self._prev_error = error

        if measured == 0.0:
            self._integral = max(0.0, self._integral)

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

        # Integral and derivative transients must not reverse an active correction.
        return output if output * self._active_direction > 0 else 0.0

    def step(self, now, dt, start_time, measured_force, deadline):
        if self.max_tension_kg is not None and measured_force >= self.max_tension_kg:
            self._stop_pulse.set()
            self._speed_changed.set()
            raise TrialLimitReached(f"{self.name} reached {measured_force:g} kg "
                                    f"(tension limit {self.max_tension_kg:g} kg)")
        if self.tightening_steps == self.max_tightening_steps:
            raise TrialLimitReached(f"{self.name} reached {self.tightening_steps} tightening steps")
        self._maybe_advance_setpoint(now, start_time)
        output = self._pid_step(measured_force, dt)
        speed = output if abs(output) >= MIN_ACTIVE_STEPS_PER_SEC else 0.0
        command = (speed, deadline)
        if command != self._pulse_command:
            self._pulse_command = command
            self._speed_changed.set()

        self.log_times.append(now - start_time)
        self.log_measured.append(measured_force)
        self.log_setpoints.append(self.current_setpoint)
        self.log_outputs.append(output)

        return measured_force, output

    def stop(self):
        self._stop_pulse.set()
        self._speed_changed.set()
        if self._pulse_thread.ident is not None:
            self._pulse_thread.join()
        motors.stop_motor([self.motor_index])


LOG_CSV_TEMPLATE = "pid_force_log_{name}.csv"
COMBINED_PLOT_PNG = "pid_force_plot_all_motors.png"

CHANNEL_COLORS = ["#3B6FA0", "#D9534F", "#5CB85C", "#F0AD4E", "#9B59B6", "#1ABC9C"]


def save_log_csv(channel, csv_path=None):
    """Writes one channel's run to its own CSV file."""
    if not channel.log_times:
        print(f"[{channel.name}] No data logged -- skipping CSV.")
        return

    if csv_path is None:
        csv_path = LOG_CSV_TEMPLATE.format(name=channel.name)
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["elapsed_s", "measured_force_kg", "setpoint_kg", "output_steps_per_s"])
        writer.writerows(zip(channel.log_times, channel.log_measured,
                              channel.log_setpoints, channel.log_outputs))
    print(f"[{channel.name}] Saved run log to {csv_path}")


def save_combined_plot(channels, plot_path=COMBINED_PLOT_PNG):
    """One figure with every channel's measured force + setpoint overlaid,
    instead of a separate PNG per motor. Works for any number of channels
    given in channels."""
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
    ax.set_ylim(-0.5,3)
    ax.set_title("PID force control: measured vs. setpoint")
    ax.legend(loc="best", frameon=False, ncol=2, fontsize=8)
    ax.grid(True, linestyle="--", alpha=0.4)
    for spine in ["top", "right"]:
        ax.spines[spine].set_visible(False)
    plt.tight_layout()
    plt.savefig(plot_path)
    plt.close(fig)
    print(f"Saved plot to {plot_path}")


def main(motor=None, *, tuning=None, duration_s=None):
    configs = CHANNEL_CONFIGS if motor is None else (CHANNEL_CONFIGS[motor],)
    if tuning is not None:
        configs = (dict(CHANNEL_CONFIGS[motor], **tuning),)
    channels = [MotorChannel(**config) for config in configs]
    plot_path = COMBINED_PLOT_PNG if motor is None else f"pid_force_plot_motor{motor}.png"
    csv_path = None
    if tuning is not None:
        trial = f"pid_tune_motor{motor}_{time.strftime('%Y%m%d-%H%M%S')}"
        plot_path = f"{trial}.png"
        csv_path = f"{trial}.csv"
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as stream:
            stream.settimeout(0.1)
            stream.connect(MONITOR_ADDRESS)
            startup_deadline = time.monotonic() + 5.0
            while True:
                try:
                    fetch_loads(stream, channels)
                    break
                except RuntimeError:
                    if time.monotonic() >= startup_deadline:
                        raise
                    time.sleep(0.05)

            start_time = last_pid_time = last_print_time = time.monotonic()
            for ch in channels:
                ch.start(start_time)

            end_time = start_time + duration_s if duration_s is not None else math.inf
            while True:
                next_update = min(last_pid_time + PID_DT, end_time)
                time.sleep(max(0.0, next_update - time.monotonic()))
                if time.monotonic() >= end_time:
                    break
                loads = fetch_loads(stream, channels)
                now = time.monotonic()
                if now >= end_time:
                    break
                dt = now - last_pid_time
                last_pid_time = now
                for ch, (force, deadline) in zip(channels, loads):
                    pulse_deadline = int(min(deadline, end_time * 1e9))
                    ch.step(now, dt, start_time, force, pulse_deadline)

                if now - last_print_time >= 0.2:
                    print("  |  ".join(
                        f"{ch.name}: force={ch.log_measured[-1]:.3f}kg "
                        f"setpoint={ch.current_setpoint}kg output={ch.log_outputs[-1]:.1f} steps/s"
                        for ch in channels
                    ))
                    last_print_time = now

    except TrialLimitReached as error:
        print(f"\nTrial stopped: {error}")
    except KeyboardInterrupt:
        print("\nInterrupted, stopping.")
    finally:
        for ch in channels:
            ch.stop()
        motors.close()
        GPIO.cleanup()
        for ch in channels:
            save_log_csv(ch, csv_path)
        save_combined_plot(channels, plot_path)

    return channels


if __name__ == "__main__":
    main(args.motor)


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
