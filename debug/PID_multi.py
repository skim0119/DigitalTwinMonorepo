"""Run load-cell force control with native C++ PID workers."""

import argparse
import csv
import json
import math
import os
import socket
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from tempfile import mkdtemp


DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "pins.json"
OUTPUT_DIR = DEFAULT_CONFIG.parent / "output"
TELEMETRY_INTERVAL = 0.065


TERMINAL_IDLE_S = 5.0


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--motor", type=int, choices=range(6),
                        help="Run only this motor and its HX711 (default: all motors).")
    selection.add_argument("--skip-motor", type=int, choices=range(6),
                           help="Run all other motors and ignore this motor’s HX711.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG,
                        help="Use an alternate pins.json-format file.")
    parser.add_argument("--terminal", action="store_true",
                        help="Stop after five seconds without motor activity.")
    return parser.parse_args()


@dataclass
class ChannelLog:
    name: str
    kp: float
    ki: float
    kd: float
    tightening_steps: int = 0
    log_times: list = field(default_factory=list)
    log_measured: list = field(default_factory=list)
    log_setpoints: list = field(default_factory=list)
    log_terms: list = field(default_factory=list)
    log_outputs: list = field(default_factory=list)


LOG_CSV_TEMPLATE = "pid_force_log_{name}.csv"
COMBINED_PLOT_PNG = "pid_force_plot_all_motors.png"

CHANNEL_COLORS = ["#3B6FA0", "#D9534F", "#5CB85C", "#F0AD4E", "#9B59B6", "#1ABC9C"]


class SensorRecorder:
    """Record the streamer's six unfiltered loads alongside commanded motor steps."""

    def __init__(self, path, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.socket.connect(("127.0.0.1", port))
        try:
            self.file = path.open("x", newline="")
        except BaseException:
            self.socket.close()
            raise
        self.writer = csv.writer(self.file)
        self.writer.writerow(["elapsed_s", *(f"load_{i}_kgf" for i in range(6)),
                              *(f"load_{i}_age_s" for i in range(6)),
                              *(f"motor_{i}_commanded_steps" for i in range(6))])
        self.readings = [(0, 0.0)] * 6

    def sample(self, elapsed, now, motion):
        try:
            while True:
                lines = self.socket.recv(512).decode("ascii").splitlines()
                readings = [(int(stamp), float(load)) for stamp, load in (line.split() for line in lines)]
                if len(readings) == 6 and all(stamp >= 0 and math.isfinite(load) for stamp, load in readings):
                    self.readings = readings
        except (BlockingIOError, ConnectionRefusedError, UnicodeError, ValueError):
            pass
        try:
            self.socket.send(b"hx711-fast")
        except ConnectionRefusedError:
            pass
        ages = [now - stamp / 1e9 if stamp and 0 <= now - stamp / 1e9 else ""
                for stamp, _ in self.readings]
        loads = [load if age != "" and age <= 1.0 else ""
                 for age, (_, load) in zip(ages, self.readings)]
        self.writer.writerow([elapsed, *loads, *ages, *(status[1] for status in motion)])

    def close(self):
        self.socket.close()
        self.file.close()


def save_log_csv(channel, csv_path):
    """Writes one channel's run to its own CSV file."""
    if not channel.log_times:
        print(f"[{channel.name}] No data logged -- skipping CSV.")
        return

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["elapsed_s", "measured_force_kg", "setpoint_kg", "output_steps_per_s",
                         "p_steps_per_s", "i_steps_per_s", "d_steps_per_s", "ff_steps_per_s",
                         "requested_steps_per_s", "filtered_steps_per_s"])
        for t, force, target, output, terms in zip(channel.log_times, channel.log_measured,
                                                  channel.log_setpoints, channel.log_outputs, channel.log_terms):
            writer.writerow([t, force, target, output, *terms])
    print(f"[{channel.name}] Saved run log to {csv_path}")


def save_combined_plot(channels, plot_path):
    """Plot force tracking and commanded step rates on a shared time axis."""
    channels_with_data = [ch for ch in channels if ch.log_times]
    if not channels_with_data:
        print("No data logged for any channel -- skipping combined plot.")
        return

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax, output_ax) = plt.subplots(2, 1, sharex=True, figsize=(10, 8), dpi=150)
    for i, ch in enumerate(channels_with_data):
        color = CHANNEL_COLORS[i % len(CHANNEL_COLORS)]
        ax.plot(ch.log_times, ch.log_measured, color=color, linewidth=1.5,
                label=f"{ch.name} force")
        ax.plot(ch.log_times, ch.log_setpoints, color=color, linestyle="--",
                linewidth=1.2, alpha=0.6, label=f"{ch.name} setpoint")
        output_ax.plot(ch.log_times, ch.log_outputs, color=color, linewidth=1.2,
                       label=ch.name)

    output_ax.set_xlabel("Time (s)")
    output_ax.set_ylabel("Output (steps/s)")
    output_ax.set_title("Commanded motor step rate")
    ax.set_ylabel("Force (kg)")
    ax.set_title("PID force control: measured vs. setpoint")
    for axis in (ax, output_ax):
        axis.legend(loc="best", frameon=False, ncol=2, fontsize=8)
        axis.grid(True, linestyle="--", alpha=0.4)
        for spine in ("top", "right"):
            axis.spines[spine].set_visible(False)
    plt.tight_layout()
    plt.savefig(plot_path)
    plt.close(fig)
    print(f"Saved plot to {plot_path}")


def main(motor=None, *, skip_motor=None, tuning=None, duration_s=None, config=DEFAULT_CONFIG, terminal=False):
    from control_core import open_controller, safety_limits

    if motor is not None and motor not in range(6):
        raise ValueError("Motor must be between 0 and 5")
    if skip_motor is not None and (skip_motor not in range(6) or motor is not None):
        raise ValueError("Skip motor must be between 0 and 5 and used without a single motor")
    if tuning is not None and motor is None:
        raise ValueError("A tuning trial requires a single motor")
    if duration_s is not None and (not math.isfinite(duration_s) or duration_s <= 0):
        raise ValueError("Trial duration must be positive and finite")

    settings = json.loads(Path(config).read_text())
    force_limit, max_rate = safety_limits(settings)
    overrides = tuning or {}
    selected = tuple(index for index in range(6) if index != skip_motor) if motor is None else (motor,)
    schedule = tuple(overrides.get("setpoint_schedule", [0.1]))
    if not schedule or (len(schedule) > 1 and duration_s is None):
        raise ValueError("A multi-target schedule requires a finite trial duration")
    target = schedule[0]
    hold_s = math.inf if duration_s is None else duration_s / len(schedule)
    tension_limit = overrides.get("max_tension_kg")
    tightening_limit = overrides.get("max_tightening_steps")
    if tension_limit is None:
        tension_limit = force_limit
    if not 0 < tension_limit <= force_limit:
        raise ValueError(f"Tension limit must be positive and at most {force_limit:g} kgf")
    if any(not math.isfinite(value) or value < 0 or
           value >= tension_limit for value in schedule):
        raise ValueError("Target must be finite, nonnegative, and below the tension limit")
    if overrides.get("deadband_g") is not None:
        settings["pid_deadband_g"] = overrides["deadband_g"]
    channels = []
    for index in selected:
        gains = settings["motors"][index]["pid"]
        gains.update({key: overrides[key] for key in ("kp", "ki", "kd") if key in overrides})
        channels.append(ChannelLog(f"motor{index}", *(gains[key] for key in ("kp", "ki", "kd"))))

    settings.setdefault("pid_filter_alpha", 0.4)
    timestamp = datetime.now(timezone.utc)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    run_dir = Path(mkdtemp(prefix=timestamp.strftime("%Y%m%dT%H%M%S.%fZ-"), dir=OUTPUT_DIR))
    metadata = {
        "started_at_utc": timestamp.isoformat(),
        "config_file": str(Path(config).resolve()),
        "motors": selected,
        "target_kg": target,
        "setpoint_schedule_kg": schedule,
        "duration_s": duration_s,
        "terminal": terminal,
        "max_tension_kg": tension_limit,
        "max_tightening_steps": tightening_limit,
        "max_step_rate": max_rate,
        "settings": settings,
    }
    (run_dir / "run.json").write_text(json.dumps(metadata, indent=2, allow_nan=False) + "\n")
    print(f"Run output: {run_dir}")

    port = int(os.getenv("SPIROB_MONITOR_PORT", "5050"))
    controller = open_controller(
        settings, [(-sys.maxsize, sys.maxsize)] * 6,
        port=port,
        max_force=tension_limit,
        max_rate=max_rate, motor=motor, skip_motor=skip_motor,
        max_tightening_steps=0 if tightening_limit is None else tightening_limit,
    )
    targets = [target if index in selected else 0.0 for index in range(6)]
    recorder = None
    try:
        if tuning is not None:
            recorder = SensorRecorder(run_dir / "all_sensors.csv", port)
        startup_deadline = time.monotonic() + 5.0
        while not controller.ready():
            if time.monotonic() >= startup_deadline:
                raise RuntimeError("No fresh HX711 readings. Start the updated GPIO streamer.")
            time.sleep(0.02)

        start = time.monotonic()
        end = math.inf if duration_s is None else start + duration_s
        last_print = start
        idle_since = start
        previous_steps = None
        while (now := time.monotonic()) < end:
            elapsed = now - start
            phase = min(int(elapsed / hold_s), len(schedule) - 1)
            target = schedule[phase]
            if target != targets[selected[0]]:
                for index in selected:
                    targets[index] = target
            controller.command(targets, True, min(end - now, 2 * TELEMETRY_INTERVAL))
            status = controller.pid_status()
            if recorder is not None:
                recorder.sample(elapsed, now, controller.status())
            for index, channel in zip(selected, channels):
                force, speed, *terms = status[index]
                channel.log_terms.append(terms)
                channel.log_times.append(elapsed)
                channel.log_measured.append(force)
                channel.log_setpoints.append(target)
                channel.log_outputs.append(speed)
            if terminal and phase == len(schedule) - 1:
                motion = controller.status()
                steps = tuple(motion[index][1] for index in selected)
                if steps != previous_steps or any(motion[index][2] != 0 for index in selected):
                    idle_since = now
                previous_steps = steps
                if now - idle_since >= TERMINAL_IDLE_S:
                    print("No motor activity for 5 seconds, stopping.")
                    break
            if now - last_print >= 0.2:
                print("  |  ".join(
                    f"{channel.name}: force={channel.log_measured[-1]:.3f}kg "
                    f"setpoint={target:g}kg output={channel.log_outputs[-1]:.1f} steps/s"
                    for channel in channels
                ))
                last_print = now
            time.sleep(min(TELEMETRY_INTERVAL, max(0, end - time.monotonic())))
    except KeyboardInterrupt:
        print("\nInterrupted, stopping.")
    except RuntimeError as error:
        message = str(error)
        if (not message.startswith("Measured tension reached the configured limit")
                and message != "Tightening STEP limit reached"):
            raise
        print(f"\nTrial stopped: {message}")
    finally:
        controller.stop()
        if recorder is not None:
            recorder.close()
        for index, channel in zip(selected, channels):
            channel.tightening_steps = controller.tightening_steps(index)
            save_log_csv(channel, run_dir / LOG_CSV_TEMPLATE.format(name=channel.name))
        plot_name = COMBINED_PLOT_PNG if motor is None else f"pid_force_plot_motor{motor}.png"
        save_combined_plot(channels, run_dir / plot_name)
    return channels


if __name__ == "__main__":
    args = parse_args()
    main(args.motor, skip_motor=args.skip_motor, config=args.config, terminal=args.terminal)
