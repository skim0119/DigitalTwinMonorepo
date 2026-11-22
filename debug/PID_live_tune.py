"""Graphical hand tuning for one Spirob motor and load cell."""

import argparse
import json
import math
import os
import sys
import time
from collections import deque
from pathlib import Path

from PID_multi import DEFAULT_CONFIG

REFRESH_MS = 50
WINDOW_S = 15


class LiveTuner:
    def __init__(self, figure, settings, motor, target, max_tension, max_steps, open_controller):
        from matplotlib.widgets import Button, TextBox

        self.figure = figure
        self.controller = None
        self.running = False
        self.motor = motor
        self.max_tension = max_tension
        self.connect_controller = lambda: open_controller(
            settings, [(-sys.maxsize, sys.maxsize)] * 6,
            port=int(os.getenv("SPIROB_MONITOR_PORT", "5050")),
            max_force=max_tension,
            motor=motor, max_tightening_steps=max_steps,
        )
        self.origin = time.monotonic()
        self.points = deque(maxlen=WINDOW_S * 1000 // REFRESH_MS + 1)
        self.target = target
        pid = settings["motors"][motor]["pid"]
        self.gains = tuple(pid[key] for key in ("kp", "ki", "kd"))

        figure.canvas.manager.set_window_title(f"Spirob PID tuner · Motor {motor}")
        figure.set_facecolor("#f1f4f6")
        figure.text(0.075, 0.92, f"Motor {motor} · Live PID tuning", size=17,
                    color="#132c3b", weight="bold")
        self.status = figure.text(0.075, 0.855, "Disconnected", color="#324e60", size=10, wrap=True)

        self.axes = figure.add_axes([0.09, 0.20, 0.59, 0.57], facecolor="white")
        self.axes.set(xlabel="Time relative to now (s)", ylabel="Force (kgf)",
                      xlim=(-WINDOW_S, 0), ylim=(0, max(target, 0.01) * 1.2))
        self.axes.grid(color="#dce4e9", linewidth=0.7)
        self.axes.set_axisbelow(True)
        self.load, = self.axes.plot([], [], color="#066d78", linewidth=2, label="Measured load")
        self.setpoint, = self.axes.plot([], [], color="#9c4514", linestyle="--",
                                       drawstyle="steps-post", label="Target")
        self.axes.legend(loc="upper left", frameon=False)

        self.fields = []
        for index, (label, value) in enumerate(zip(("Target (kgf)", "Kp", "Ki", "Kd"), (target, *self.gains))):
            bottom = 0.66 - index * 0.11
            figure.text(0.74, bottom + 0.067, label, size=10, color="#132c3b")
            axes = figure.add_axes([0.74, bottom, 0.20, 0.052])
            field = TextBox(axes, "", initial=str(value), color="white", hovercolor="#e7f1f2")
            self.fields.append(field)

        self.apply_button = Button(figure.add_axes([0.74, 0.265, 0.20, 0.058]),
                                   "Apply settings", color="#c9e6e9", hovercolor="#acd8dc")
        self.start_button = Button(figure.add_axes([0.74, 0.186, 0.095, 0.055]),
                                   "Connect", color="#c9e6e9", hovercolor="#acd8dc")
        self.stop_button = Button(figure.add_axes([0.845, 0.186, 0.095, 0.055]),
                                  "Stop", color="#f3dcd0", hovercolor="#ecc3af")
        self.apply_button.on_clicked(self.apply)
        self.start_button.on_clicked(self.start)
        self.stop_button.on_clicked(self.stop)
        figure.text(0.075, 0.065,
                    f"Stop limits   {max_tension:g} kgf measured tension  /  {max_steps} tightening pulses per session",
                    color="#324e60", size=10)
        self.timer = figure.canvas.new_timer(interval=REFRESH_MS)
        self.timer.add_callback(self.tick)
        figure.canvas.mpl_connect("close_event", self.shutdown)
        self.timer.start()

    def message(self, text):
        self.status.set_text(text)
        self.figure.canvas.draw_idle()

    def apply(self, _event=None):
        try:
            target, *gains = (float(field.text) for field in self.fields)
            if not all(math.isfinite(value) and value >= 0 for value in (target, *gains)):
                raise ValueError
            if target >= self.max_tension:
                self.message("Target must be below the measured-tension limit.")
                return False
        except ValueError:
            self.message("Enter finite, nonnegative values for the target and all three gains.")
            return False
        self.target, self.gains = target, tuple(gains)
        self.message("Settings applied to running motor." if self.running else "Settings ready for the next start.")
        return True

    def connect(self):
        try:
            self.controller = self.connect_controller()
        except (OSError, ValueError, RuntimeError) as error:
            self.message(str(error))
            return
        self.start_button.label.set_text("Start")
        self.message("Waiting for live load readings…")

    def start(self, _event=None):
        if self.running:
            return
        if self.controller is None:
            self.connect()
            return
        try:
            if not self.controller.ready():
                self.message("Waiting for HX711 readings. Check that the Pi streamer is running.")
                return
            if self.apply():
                self.running = True
                self.start_button.label.set_text("Running")
                self.message("Motor running. Apply settings to change the target or gains.")
        except RuntimeError as error:
            self.stop(reason=str(error))

    def disconnect(self):
        self.running = False
        if self.controller is not None:
            self.controller.stop()
            self.controller = None
    def stop(self, _event=None, *, reason="Motion stopped. Connect to start a new session."):
        self.disconnect()
        self.start_button.label.set_text("Connect")
        self.message(reason)

    def shutdown(self, _event=None):
        self.timer.stop()
        self.disconnect()

    def tick(self):
        if self.controller is None:
            return
        try:
            if not self.running and not self.controller.ready():
                return
            if self.running:
                targets = [0.0] * 6
                targets[self.motor] = self.target
                self.controller.command(targets, True, 0.2, gains=self.gains)
            elif self.status.get_text() == "Waiting for live load readings…":
                self.message("Load readings live. Press Start to enable motion.")
            force, _, _ = self.controller.status()[self.motor]
        except (RuntimeError, TypeError, ValueError) as error:
            self.stop(reason=f"Motion stopped. {error}")
            return

        elapsed = time.monotonic() - self.origin
        self.points.append((elapsed, force, self.target))
        while self.points[0][0] < elapsed - WINDOW_S:
            self.points.popleft()
        times, readings, targets = zip(*self.points)
        relative = [stamp - elapsed for stamp in times]
        self.load.set_data(relative, readings)
        self.setpoint.set_data(relative, targets)
        low, high = min(0, *readings), max(0.01, *readings, *targets)
        self.axes.set_ylim(low - (high - low) * 0.08, high + (high - low) * 0.1)
        self.figure.canvas.draw_idle()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motor", type=int, choices=range(6), required=True)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--target-kg", type=float, default=0.1)
    parser.add_argument("--max-tension-kg", type=float, required=True)
    parser.add_argument("--max-tightening-steps", type=int, required=True)
    args = parser.parse_args()
    if not (math.isfinite(args.max_tension_kg) and 0 <= args.target_kg < args.max_tension_kg
            and args.max_tightening_steps > 0):
        parser.error("Set a finite tension limit above the target and a positive tightening pulse limit.")

    import matplotlib.pyplot as plt
    from control_core import open_controller

    settings = json.loads(args.config.read_text())
    figure = plt.figure(figsize=(10.5, 6.2))
    if figure.canvas.required_interactive_framework is None:
        parser.exit(1, "A graphical Matplotlib backend is required. Run from a desktop session.\n")
    app = LiveTuner(figure, settings, args.motor, args.target_kg,
                    args.max_tension_kg, args.max_tightening_steps, open_controller)
    try:
        app.connect()
        plt.show()
    finally:
        app.shutdown()


if __name__ == "__main__":
    main()
