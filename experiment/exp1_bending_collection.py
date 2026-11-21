"""Collect repeated quasi-static single-tendon force staircases on Spirob."""

import argparse
import csv
import json
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from urllib.error import HTTPError
from urllib.parse import urlencode
from urllib.request import Request, urlopen

if __package__:
    from .quasi_static import TENDONS, force_staircase
else:
    from quasi_static import TENDONS, force_staircase

BODIES = ("arm_base", "arm_tip")
ROOT = Path(__file__).resolve().parents[1]
ACTIVE = {"queued", "running", "stopping"}
FIELDS = ("cycle", "elapsed_s", "ros_stamp_ns", "frame", "body",
          "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw",
          "tendon", "target_kgf", "load_kgf", "run_status", "simulated")


def make_table(tendon, force, cycles, steps, dwell, *, step_kgf=0.2):
    return force_staircase((tendon,), ((0,), (force,), (0,)), cycles, steps, dwell, step_kgf=step_kgf)


def make_pair_table(force, cycles, steps, dwell, *, step_kgf=0.2):
    waypoints = ((0, 0), (force, 0), (force, force), (0, force), (0, 0))
    return force_staircase((0, 1), waypoints, cycles, steps, dwell, step_kgf=step_kgf)


def export(path, run, poses, controls, tendons, cycle_duration, cycles, bodies=BODIES):
    loads = iter(row for row in controls if row[1] in tendons)
    upcoming = next(loads, None)
    latest = {}
    recorded = set()
    cycle = 0
    with path.open("x", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(FIELDS)
        for elapsed, stamp, frame, body, *pose in poses:
            if body not in bodies:
                continue
            cycle = min(int(elapsed / cycle_duration), cycles - 1)
            while upcoming is not None and upcoming[0] <= elapsed:
                latest[upcoming[1]] = (upcoming[2], upcoming[4])
                upcoming = next(loads, None)
            for tendon in tendons:
                target, load = latest.get(tendon, (0, ""))
                writer.writerow((cycle + 1, elapsed - cycle * cycle_duration, stamp, frame,
                                 body, *pose, tendon, target, load, run["status"], run["simulated"]))
            recorded.add((cycle, body))
    return all((index, body) in recorded for index in range(cycles) for body in bodies)


def collect_local(raw, path, tendons, cycle_duration, cycles, args):
    import fcntl

    sys.path.insert(0, str(ROOT / "experiment-api"))
    from control_core import open_controller
    from experiment_api.hardware import Hardware
    from experiment_api.runner import Experiments
    from experiment_api.table import parse_table

    table = parse_table(raw, "force", "kgf")
    bodies = tuple(dict.fromkeys(args.name or BODIES))

    class ForcePID(Hardware):
        def validate(self, table):
            if table.mode != "force" or table.unit != "kgf":
                raise ValueError("Exp 1 requires force commands in kgf.")
            if max(map(max, table.values)) >= args.max_tension_kg:
                raise ValueError("Every commanded force must be below --max-tension-kg.")

        def open(self, _table, cancel):
            self.controller = open_controller(
                args.config, [(-sys.maxsize, sys.maxsize)] * len(TENDONS),
                port=int(os.environ.get("SPIROB_MONITOR_PORT", "5050")),
                max_force=args.max_tension_kg, max_rate=2133,
                max_tightening_steps=args.max_tightening_steps,
            )
            self.wait_ready(cancel)

    with (Path.home() / ".spirob-experiment-hardware.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("The Experiment API or another collector owns the hardware.") from None
        hardware = ForcePID(None)
        hardware.validate(table)
        experiment = Experiments(path.with_suffix(""), hardware)
        stopped = False
        run = None
        try:
            experiment.start_mocap()
            deadline = time.monotonic() + 3
            while not experiment.mocap_error and time.monotonic() < deadline:
                with experiment.lock:
                    if all(time.monotonic() - experiment.last_body_pose.get(body, -math.inf) <= 1
                           for body in bodies):
                        break
                time.sleep(0.05)
            run, = experiment.submit(table, raw, "Exp 1 bending collection", 1, bodies)
            print(f"Recording run {run['id']}", flush=True)
            while experiment.worker.is_alive():
                time.sleep(0.1)
        except KeyboardInterrupt:
            stopped = True
        finally:
            experiment.close()

    if run is None:
        return 130
    run = experiment.get(run["id"])
    folder = experiment.root / run["id"]

    def read_rows(name):
        source = folder / name
        if source.is_file():
            with source.open(newline="") as stream:
                rows = csv.reader(stream)
                next(rows, None)
                for elapsed, *values in rows:
                    yield (float(elapsed), *values)

    complete = export(path, run, read_rows("poses.csv"), read_rows("controls.csv"),
                      tendons, cycle_duration, cycles, bodies)
    return report_result(path, run, complete, stopped)


def main(*, pair=False):
    number = 2 if pair else 1
    parser = argparse.ArgumentParser(description=f"Collect quasi-static SpiRob bending data for Exp {number}.")
    if not pair:
        parser.add_argument("-N", "--tendon", type=int, choices=range(6), default=0)
    parser.add_argument("-F", "--force-kgf", type=float, default=2.5)
    parser.add_argument("-C", "--cycles", type=int, choices=range(1, 31), default=1)
    increments = parser.add_mutually_exclusive_group()
    increments.add_argument("--step-kgf", type=float, default=0.2,
                            help="force increment in kgf (default: 0.2)")
    increments.add_argument("--steps", type=int, choices=range(1, 101),
                            help="use this many equal increments per phase")
    parser.add_argument("--dwell-s", type=float, default=1.0,
                        help="seconds to hold each setpoint (default: 1)")
    if pair:
        parser.add_argument("--api-url", default="http://127.0.0.1:5052")
    else:
        parser.add_argument("--config", type=Path,
                            default=Path(os.environ.get("SPIROB_PINS_CONFIG", ROOT / "pins.json")))
        parser.add_argument("--max-tension-kg", type=float, required=True)
        parser.add_argument("--max-tightening-steps", type=int, required=True)
        parser.add_argument("--name", action="append", help="ROS rigid body to record (repeatable)")
    args = parser.parse_args()
    if not pair and not (math.isfinite(args.max_tension_kg) and args.max_tension_kg > 0
                         and 0 < args.max_tightening_steps <= sys.maxsize):
        parser.error("Use a finite positive tension limit and a positive tightening pulse limit.")
    if not all(math.isfinite(value) and value > 0 for value in (
        args.force_kgf, args.step_kgf, args.dwell_s
    )):
        parser.error("Use finite positive force, increment, and dwell values.")

    try:
        if pair:
            raw, cycle_duration = make_pair_table(
                args.force_kgf, args.cycles, args.steps, args.dwell_s, step_kgf=args.step_kgf
            )
            tendons = TENDONS[:2]
        else:
            raw, cycle_duration = make_table(
                args.tendon, args.force_kgf, args.cycles, args.steps, args.dwell_s,
                step_kgf=args.step_kgf,
            )
            tendons = (TENDONS[args.tendon],)
    except ValueError as error:
        parser.error(str(error))

    folder = ROOT / "output" / "experiment"
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f'exp{number}_{datetime.now().strftime("%Y%m%d_%H%M%S_%f")}.csv'
    if not pair:
        return collect_local(raw, path, tendons, cycle_duration, args.cycles, args)

    def request(route, data=None):
        try:
            request = Request(args.api_url.rstrip("/") + route, data=data)
            with urlopen(request, timeout=10) as response:
                return json.load(response)
        except HTTPError as error:
            raise RuntimeError(error.read().decode()) from error

    health = request("/health")
    if not health.get("required_body_supported"):
        parser.error("Update and restart Experiment API before collecting bending data.")
    params = urlencode(dict(mode="force", unit="kgf", label=f"Exp {number} bending collection",
                            reference_ready="true",
                            required_body=BODIES), doseq=True)
    run, = request("/runs?" + params, raw)
    run_id = run["id"]
    print(f"Recording run {run_id}", flush=True)

    def wait():
        while True:
            state = next(run for run in request("/runs") if run["id"] == run_id)
            if state["status"] not in ACTIVE:
                return state
            time.sleep(0.25)

    stopped = False
    try:
        run = wait()
    except KeyboardInterrupt:
        stopped = True
        try:
            request(f"/runs/{run_id}/stop", b"")
        except RuntimeError:
            if any(run["id"] == run_id and run["status"] in ACTIVE for run in request("/runs")):
                raise
        run = wait()

    poses = request(f"/runs/{run_id}/samples?kind=poses")
    controls = request(f"/runs/{run_id}/samples?kind=controls")
    complete = export(path, run, poses, controls, tendons, cycle_duration, args.cycles)
    return report_result(path, run, complete, stopped)


def report_result(path, run, complete, stopped):
    print(path)
    if stopped:
        return 130
    if run["status"] != "complete" or not complete:
        print(run.get("error") or "Recording ended without every selected body in each cycle.", file=sys.stderr)
        return 1
    return 0


def run_cli(*, pair=False):
    try:
        sys.exit(main(pair=pair))
    except (RuntimeError, OSError, ValueError) as error:
        sys.exit(str(error))


if __name__ == "__main__":
    run_cli()
