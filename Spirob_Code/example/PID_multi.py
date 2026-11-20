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

PULSE_IDLE_POLL_S = 0.005

DOUT_PINS = [2, 5, 7, 21, 17, 12]
SCK_PINS = [3, 6, 11, 13, 20, 19]

PID_DT = 0.065  # trigger threshold -- actual dt used is measured per-cycle


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

        # --- reader-thread state (same pattern as motor0_PID.py) ---
        self._raw_lock = threading.Lock()
        self._latest_raw = raw_offset
        self._last_good_raw = raw_offset
        self._stop_reader = threading.Event()
        self.first_reading_ready = threading.Event()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)

        # --- pulse-thread state -- this is what keeps this motor's pulse
        # timing independent of the other motor's ---
        self.should_pulse = False
        self.last_output = 0.0
        self._stop_pulse = threading.Event()
        self._pulse_thread = threading.Thread(target=self._pulse_loop, daemon=True)

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
        motors.start_motor([self.motor_index])
        self._pulse_thread.start()

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

    def _pulse_loop(self):
        """Runs in its own thread, separate from the main PID loop and
        from the other motor's pulse thread. When should_pulse is True it
        calls motors.update() back to back (that call's internal
        time.sleep() releases the GIL, so this doesn't block the other
        motor's pulse thread or the main loop's PID computation). When
        should_pulse is False it just polls at PULSE_IDLE_POLL_S instead
        of spinning a full CPU core."""
        while not self._stop_pulse.is_set():
            if self.should_pulse:
                _update_start = time.time()
                motors.update([self.motor_index])
                _update_elapsed = time.time() - _update_start
                #if _update_elapsed > 0.05:
                    #print(f"[{self.name}] !! motors.update() took {_update_elapsed:.3f}s "
                    #      f"(output was {self.last_output:.1f} steps/s)")
            else:
                time.sleep(PULSE_IDLE_POLL_S)

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

        Sets self.should_pulse, which is what this channel's own pulse
        thread (_pulse_loop) checks -- it does NOT call motors.update()
        itself, so this method is fast and safe to call from the main
        PID loop."""
        if abs(output_steps_per_sec) < MIN_ACTIVE_STEPS_PER_SEC:
            # Close enough to setpoint that the PID wants to barely creep.
            # motors.update() blocks for a duration tied to the commanded
            # speed (near-zero speed -> very long block), so we
            # deliberately leave the motor un-pulsed rather than ask for a
            # speed that could stall its pulse thread for a long time.
            self.should_pulse = False
            return

        if output_steps_per_sec >= 0:
            motors.dir_motor_forward([self.motor_index])
        else:
            motors.dir_motor_backward([self.motor_index])

        speed_fraction = min(abs(output_steps_per_sec) / MAX_STEPS_PER_SEC, 1.0)
        motors.set_motor_speed([self.motor_index], speed_fraction)
        self.should_pulse = True

    def step(self, now, dt, start_time):
        """One PID update for this motor: advance its schedule, read its
        load cell, compute + apply its output, and log the sample. Does
        NOT touch motors.update() -- that's this channel's pulse thread's
        job, running concurrently. Returns (measured_force, output)."""
        self._maybe_advance_setpoint(now, start_time)

        measured_force = self._read_force_kg()
        output = self._pid_step(measured_force, dt)
        self.last_output = output
        self._apply_output(output)

        self.log_times.append(now - start_time)
        self.log_measured.append(measured_force)
        self.log_setpoints.append(self.current_setpoint)
        self.log_outputs.append(output)

        return measured_force, output

    def stop(self):
        """Stops this motor and both of its background threads."""
        motors.stop_motor([self.motor_index])
        self._stop_pulse.set()
        self._pulse_thread.join(timeout=1.0)
        self._stop_reader.set()
        self._reader_thread.join(timeout=1.0)


# ----------------------------------------------------------------------
# Configure each motor's channel -- calibration, gains, and setpoint
# schedule are all per-motor. EDIT THESE FOR YOUR TWO LOAD CELLS/MOTORS.
# ----------------------------------------------------------------------
channel0 = MotorChannel(
    name="motor0",
    motor_index=0,
    # TODO: fill in motor1's actual calibration/gains/schedule
    #cal_factor=23580,
    cal_factor=34170,
    raw_offset=170539,
    kp=550.0, ki=10.0, kd=5.0,
    setpoint_schedule=[0.1],
    #setpoint_schedule=[0.2, 2.5],
    #setpoint_schedule=[1,2.5],
    setpoint_interval_s=300,
)

channel1 = MotorChannel(
    name="motor1",
    motor_index=1,
    # TODO: fill in motor1's actual calibration/gains/schedule
    #cal_factor=25170,
    cal_factor=34170,
    raw_offset=-285116.4,
    kp=550.0, ki=10.0, kd=5.0,
    setpoint_schedule=[0.1],
    #setpoint_schedule=[0.2, 2.5],
    #setpoint_schedule=[0.1,1.5,3.0],
    setpoint_interval_s=300,
)

channel2 = MotorChannel(
    name="motor2",
    motor_index=2,
    cal_factor=39293.8,      # counts per kg -- motor0's 5-point calibration
    raw_offset=276320.0,     # motor0's directly measured 0g reading
    kp=550.0, ki=10.0, kd=5.0,
    #setpoint_schedule=[0.2,2.5],
    setpoint_schedule=[0.1],
    setpoint_interval_s=300.0,
)

channel3 = MotorChannel(
    name="motor3",
    motor_index=3,
    # TODO: fill in motor1's actual calibration/gains/schedule
    cal_factor=34824.0,
    raw_offset=10569.0,
    kp=550.0, ki=10.0, kd=5.0,
    setpoint_schedule=[0.1],
    setpoint_interval_s=300.0,
)

channel4 = MotorChannel(
    name="motor4",
    motor_index=4,
    # TODO: fill in motor1's actual calibration/gains/schedule
    cal_factor=33760.0,
    raw_offset=236592.0,
    kp=550.0, ki=10.0, kd=5.0,
    #setpoint_schedule=[2.5,0.2,2.5,0.2,2.5,0.2],
    setpoint_schedule=[0.1],
    setpoint_interval_s=300.0,
)

channel5 = MotorChannel(
    name="motor5",
    motor_index=5,
    # TODO: fill in motor1's actual calibration/gains/schedule
    cal_factor=31950.0,
    raw_offset=-170491,
    kp=550.0, ki=10.0, kd=5.0,
    #setpoint_schedule=[2.5,0.2,2.5,0.2,2.5,0.2],
    #setpoint_schedule=[0.1,0.5,0.9,1.3,1.5,1.8,2.1,2.5,2.1,1.8,1.5,1.3,0.9,0.5,0.1],
    setpoint_schedule = [0.1],
    setpoint_interval_s=300,
)


CHANNELS = [channel0, channel1, channel2, channel3, channel4, channel5]

LOG_CSV_TEMPLATE = "pid_force_log_{name}.csv"
COMBINED_PLOT_PNG = "pid_force_plot_all_motors.png"

CHANNEL_COLORS = ["#3B6FA0", "#D9534F", "#5CB85C", "#F0AD4E", "#9B59B6", "#1ABC9C"]


def save_log_csv(channel):
    """Writes one channel's run to its own CSV file."""
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
    """One figure with every channel's measured force + setpoint overlaid,
    instead of a separate PNG per motor. Works for any number of channels
    in CHANNELS, not just two or three."""
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
    ax.set_title("PID force control: all motors, measured vs. setpoint")
    ax.legend(loc="best", frameon=False, ncol=2, fontsize=8)
    ax.grid(True, linestyle="--", alpha=0.4)
    for spine in ["top", "right"]:
        ax.spines[spine].set_visible(False)
    plt.tight_layout()
    plt.savefig(COMBINED_PLOT_PNG)
    plt.close(fig)
    print(f"Saved combined plot to {COMBINED_PLOT_PNG}")


def main():
    start_time = time.time()
    for ch in CHANNELS:
        ch.start(start_time)  # starts reader thread, motor, and pulse thread

    last_pid_time = time.time()
    last_print_time = start_time

    try:
        while True:
            now = time.time()

            if now - last_pid_time >= PID_DT:
                dt = now - last_pid_time
                last_pid_time = now

                for ch in CHANNELS:
                    ch.step(now, dt, start_time)

                if now - last_print_time >= 0.2:  # print at most 5x/sec, not 50x/sec
                    line = "  |  ".join(
                        f"{ch.name}: force={ch.log_measured[-1]:.3f}kg "
                        f"setpoint={ch.current_setpoint}kg output={ch.log_outputs[-1]:.1f} steps/s"
                        + ("" if ch.should_pulse else " (below min speed, not pulsing)")
                        for ch in CHANNELS
                    )
                    print(line)
                    last_print_time = now
            # No motors.update() call here -- each channel's own pulse
            # thread handles that concurrently. The main loop's only job
            # is computing PID at PID_DT and printing status.

    except KeyboardInterrupt:
        print("\nInterrupted, stopping.")
    finally:
        for ch in CHANNELS:
            ch.stop()
        motors.close()
        GPIO.cleanup()  # releases the load cells' DOUT/SCK pins too;
                         # harmless if motors.close() already cleans up GPIO
        for ch in CHANNELS:
            #save_log_and_plot(ch)
            save_log_csv(ch)
        save_combined_plot(CHANNELS)


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
# - Requires the patched motors.py (per-motor _pulse_in_high_phase and
#   start_time, not shared scalars) -- without that fix, running two
#   motors concurrently corrupts both motors' pulse timing regardless of
#   the threading structure here.

