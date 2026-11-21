"""Collect repeated quasi-static single-tendon force staircases via Experiment API."""

import argparse
import csv
import json
import math
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
ACTIVE = {"queued", "running", "stopping"}
FIELDS = ("cycle", "elapsed_s", "ros_stamp_ns", "frame", "body",
          "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw",
          "tendon", "target_kgf", "load_kgf", "run_status", "simulated")


def make_table(tendon, force, cycles, steps, dwell, *, step_kgf=0.2):
    return force_staircase((tendon,), ((0,), (force,), (0,)), cycles, steps, dwell, step_kgf=step_kgf)


def make_pair_table(force, cycles, steps, dwell, *, step_kgf=0.2):
    waypoints = ((0, 0), (force, 0), (force, force), (0, force), (0, 0))
    return force_staircase((0, 1), waypoints, cycles, steps, dwell, step_kgf=step_kgf)


def export(path, run, poses, controls, tendons, cycle_duration, cycles):
    loads = iter(row for row in controls if row[1] in tendons)
    upcoming = next(loads, None)
    latest = {}
    recorded = set()
    cycle = 0
    with path.open("x", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(FIELDS)
        for elapsed, stamp, frame, body, *pose in poses:
            if body not in BODIES:
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
    return all((index, body) in recorded for index in range(cycles) for body in BODIES)


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
    parser.add_argument("--api-url", default="http://127.0.0.1:5052")
    parser.add_argument("--reference-ready", action="store_true",
                        help="confirm calibrated mechanical zero and preload for hardware")
    args = parser.parse_args()
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
    if not health["simulated"] and not args.reference_ready:
        parser.error("Confirm the calibrated mechanical zero and preload with --reference-ready.")
    params = urlencode(dict(mode="force", unit="kgf", label=f"Exp {number} bending collection",
                            reference_ready=str(args.reference_ready).lower(),
                            required_body=BODIES), doseq=True)
    folder = Path(__file__).resolve().parents[1] / "output" / "experiment"
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f'exp{number}_{datetime.now().strftime("%Y%m%d_%H%M%S_%f")}.csv'
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
    print(path)
    if stopped:
        return 130
    if run["status"] != "complete" or not complete:
        print(run.get("error") or "Recording ended without both bodies in every cycle.", file=sys.stderr)
        return 1
    return 0


def run_cli(*, pair=False):
    try:
        sys.exit(main(pair=pair))
    except (RuntimeError, OSError) as error:
        sys.exit(str(error))


if __name__ == "__main__":
    run_cli()
