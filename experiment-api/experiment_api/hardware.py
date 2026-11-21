"""Adapt experiment tables to the native Pi controller."""
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path

from .table import TENDONS

GRAVITY = 9.80665


@dataclass(frozen=True)
class Calibration:
    steps_per_mm: tuple
    travel_mm: tuple
    max_force_kgf: float
    max_step_rate: float

    @classmethod
    def load(cls, config):
        scales = tuple(float(value) for value in config["steps_per_mm"])
        travel = tuple(tuple(map(float, bounds)) for bounds in config["travel_mm"])
        force, rate = float(config["max_force_kgf"]), float(config["max_step_rate"])
        if len(scales) != 6 or not all(math.isfinite(x) and x != 0 for x in scales):
            raise ValueError("Supply six signed, nonzero steps_per_mm calibrations.")
        if len(travel) != 6 or any(len(x) != 2 or not all(map(math.isfinite, x))
                                    or not x[0] <= 0 <= x[1] or x[0] == x[1] for x in travel):
            raise ValueError("Supply six finite travel ranges that contain zero.")
        if not math.isfinite(force) or not math.isfinite(rate) or not (0 < force and 0 < rate <= 2133):
            raise ValueError("Supply positive verified tension and step-rate limits.")
        for scale, bounds in zip(scales, travel):
            low, high = sorted(value * scale for value in bounds)
            if not -2**31 <= low < high < 2**31 or math.ceil(low) >= math.floor(high):
                raise ValueError("Each calibrated range must cover at least one full motor step.")
        return cls(scales, travel, force, rate)

    def validate(self, table):
        if table.mode == "force":
            if max(map(max, table.values)) / (GRAVITY if table.unit == "N" else 1) >= self.max_force_kgf:
                raise ValueError("A force command meets or exceeds the configured tension limit.")
            return
        if any(table.values[0]) or any(table.values[-1]):
            raise ValueError("Live displacement trials must start and end at the zero reference.")
        for i, (low, high) in enumerate(self.travel_mm):
            if any(not low <= row[i] <= high for row in table.values):
                raise ValueError(f"{TENDONS[i]} exceeds the configured travel range.")
            for j in range(1, len(table.times)):
                delta = abs((table.values[j][i] - table.values[j - 1][i]) * self.steps_per_mm[i])
                if delta / (table.times[j] - table.times[j - 1]) >= self.max_step_rate:
                    raise ValueError(f"{TENDONS[i]} exceeds the configured travel speed.")


class Hardware:
    """Bridge validated physical units to the native control worker."""

    interval = 0.025

    def __init__(self, calibration, *, open_controller=None):
        self.calibration = calibration
        self.open_controller = open_controller
        self.controller = None
        self.reference_valid = True

    def validate(self, table):
        self.calibration.validate(table)

    def open(self, _table, cancel):
        if not self.reference_valid:
            raise RuntimeError("Re-establish the mechanical zero and restart Experiment API before another trial.")
        if self.open_controller is None:
            from control_core import open_controller
            self.open_controller = open_controller
        config = self.calibration
        self.bounds = [
            (math.ceil(low), math.floor(high))
            for low, high in (sorted(x * scale for x in travel)
                              for travel, scale in zip(config.travel_mm, config.steps_per_mm))
        ]
        self.controller = self.open_controller(
            Path(os.environ.get("SPIROB_PINS_CONFIG", "pins.json")), self.bounds,
            port=int(os.environ.get("SPIROB_MONITOR_PORT", "5050")),
            max_force=config.max_force_kgf, max_rate=config.max_step_rate,
        )
        self.wait_ready(cancel)

    def wait_ready(self, cancel):
        deadline = time.monotonic() + 3
        while not cancel.is_set() and not self.controller.ready():
            if time.monotonic() >= deadline:
                raise RuntimeError("No fresh HX711 readings. Start the updated GPIO streamer.")
            cancel.wait(0.025)

    def update(self, table, values, _now, _dt, end):
        if table.mode == "force":
            targets = [x / (GRAVITY if table.unit == "N" else 1) for x in values]
        else:
            targets = [min(high, max(low, round(x * scale)))
                       for x, scale, (low, high) in zip(values, self.calibration.steps_per_mm, self.bounds)]
        remaining = end - time.monotonic()
        if remaining <= 0:
            return None
        self.controller.command(targets, table.mode == "force", min(0.15, remaining))
        return self.controller.status()

    def finish(self):
        if self.controller:
            controller, self.controller = self.controller, None
            controller.stop()
            self.reference_valid = all(steps == 0 for _, steps, _ in controller.status())

    close = finish
