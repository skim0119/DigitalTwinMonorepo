"""Jog SpiRob with an Xbox controller and record a displacement actuation CSV."""

import csv
import json
import math
import time
from pathlib import Path

from xbox_controller_6motors import main


class Recording:
    def __init__(self):
        self.rows = []

    def add_arguments(self, parser):
        parser.add_argument("--calibration", type=Path, help="Experiment calibration JSON")
        parser.add_argument("--output", type=Path, help="New displacement CSV to write on exit")

    def configure(self, parser, args):
        if args.calibration is None or args.output is None:
            parser.error("Recording requires --calibration and --output.")
        if args.output.exists() or not args.output.parent.is_dir():
            parser.error("--output must be a new file in an existing directory.")
        try:
            scales = json.loads(args.calibration.read_text())["steps_per_mm"]
            self.scales = tuple(float(scale) for scale in scales)
        except (OSError, KeyError, TypeError, ValueError, OverflowError) as error:
            parser.error(f"Cannot read steps_per_mm from calibration: {error}")
        if len(self.scales) != 6 or any(not math.isfinite(scale) or scale == 0 for scale in self.scales):
            parser.error("Calibration must contain six finite, signed, nonzero steps_per_mm values.")
        self.output = args.output

    def start(self, motors):
        self.motors = motors
        self.zero = tuple(status[1] for status in motors.status())
        self.started = time.monotonic()
        self.rows.append((0, *([0] * 6)))
        print("Recording displacement at up to 10 Hz. Return the tendons to their starting positions before exiting.", flush=True)
        return self.pause

    def sample(self, elapsed):
        pull = tuple(round((status[1] - zero) / scale, 6)
                     for status, zero, scale in zip(self.motors.status(), self.zero, self.scales))
        self.rows.append((elapsed, *pull))

    def pause(self, seconds):
        time.sleep(seconds)
        elapsed = time.monotonic() - self.started
        if elapsed - self.rows[-1][0] >= 0.1:
            self.sample(elapsed)
        # Reserve a row for the final position within the simulator's 2000-row limit.
        if len(self.rows) == 1999:
            print("Recording reached the CSV row limit. Stopping the session.", flush=True)
            raise SystemExit

    def finish(self):
        if not self.rows:
            return
        elapsed = max(time.monotonic() - self.started, self.rows[-1][0] + 0.000001)
        self.sample(elapsed)
        if elapsed > 600 or any(not 0 <= value <= 300 for _, *pull in self.rows for value in pull):
            raise SystemExit("CSV not saved: shared displacement files require pulls from 0 to 300 mm and a duration of at most 600 s.")
        with self.output.open("x", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["time_s", *(f"tendon_{i}" for i in range(1, 7))])
            writer.writerows((f"{stamp:.6f}", *pull) for stamp, *pull in self.rows)
        print(f"Saved {len(self.rows)} displacement rows to {self.output}")
        if any(self.rows[-1][1:]):
            print("Live experiment playback requires the final displacement row to be zero.")


if __name__ == "__main__":
    main(Recording())
