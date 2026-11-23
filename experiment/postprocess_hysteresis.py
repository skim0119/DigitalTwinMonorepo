"""Export complete Exp 1 cycles in the moving arm-base coordinate frame."""

import argparse
import csv
import json
import math
from bisect import bisect_right
from collections import defaultdict
from pathlib import Path

TENDONS = tuple(f"tendon_{i}" for i in range(1, 7))
POSE = ("x_m", "y_m", "z_m", "qx", "qy", "qz", "qw")
MAX_GAP_S = .25


def product(a, b):
    x, y, z, w = a
    u, v, s, t = b
    return (w*u + x*t + y*s - z*v, w*v - x*s + y*t + z*u,
            w*s + x*v - y*u + z*t, w*t - x*u - y*v - z*s)


def unit(q):
    length = math.hypot(*q)
    return tuple(v / length for v in q)


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def export_run(path, run, output):
    table = run["table"]
    times, values = table["times"], table["values"]
    active = [i for i in range(6) if len({row[i] for row in values}) > 1]
    if table["mode"] != "force" or len(active) != 1:
        return []
    index, = active
    target = [row[index] for row in values]
    if target[0] != 0 or target[-1] != 0:
        return []
    edges = [0.0]
    edges.extend(times[i+1] for i in range(1, len(target)-1) if target[i] == 0 < target[i-1])

    controls = defaultdict(dict)
    for row in read_csv(path.with_name("controls.csv")):
        if row["load_kgf"]:
            controls[float(row["elapsed_s"])][row["tendon"]] = row
    control_times = sorted(controls)
    frames = defaultdict(dict)
    for row in read_csv(path.with_name("poses.csv")):
        if row["body"].startswith("arm_"):
            frames[(float(row["elapsed_s"]), row["frame"], row["ros_stamp_ns"])][row["body"]] = row
    cycles = defaultdict(list)
    for (elapsed, frame, stamp), bodies in sorted(frames.items()):
        cycle = bisect_right(edges, elapsed) - 1
        sample = bisect_right(control_times, elapsed) - 1
        if cycle >= len(edges)-1 or sample < 0 or not {"arm_base", "arm_tip"} <= bodies.keys():
            continue
        control_time = control_times[sample]
        if elapsed - control_time > .10 or TENDONS[index] not in controls[control_time]:
            continue
        cycles[cycle].append((elapsed, frame, stamp, bodies, control_time))
    results = []
    for cycle, samples in sorted(cycles.items()):
        start, end = edges[cycle:cycle+2]
        if run["status"] != "complete" and (not control_times or control_times[-1] < end):
            continue
        coverage = [start, *(row[0] for row in samples), end]
        if any(b-a > MAX_GAP_S for a,b in zip(coverage, coverage[1:])):
            continue
        folder = output / TENDONS[index] / path.parent.parent.name / run["id"]
        folder.mkdir(parents=True, exist_ok=True)
        destination = folder / f"cycle_{cycle+1:03d}.csv"
        previous = {}
        with destination.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(("run_id", "run_status", "cycle", "elapsed_s", "cycle_elapsed_s",
                             "ros_stamp_ns", "source_frame", "body", *POSE, "active_tendon",
                             "control_elapsed_s", "active_target", "target_unit", "active_load_kgf",
                             *(f"{name}_load_kgf" for name in TENDONS)))
            for elapsed, frame, stamp, bodies, control_time in samples:
                base = tuple(float(bodies["arm_base"][key]) for key in POSE)
                q = unit(base[3:])
                inverse = (-q[0], -q[1], -q[2], q[3])
                loads = controls[control_time]
                active_load = loads[TENDONS[index]]
                for body, row in sorted(bodies.items()):
                    pose = tuple(float(row[key]) for key in POSE)
                    delta = tuple(a-b for a,b in zip(pose[:3], base[:3]))
                    position = product(product(inverse, (*delta, 0)), q)[:3]
                    orientation = product(inverse, unit(pose[3:]))
                    if body in previous and sum(a*b for a,b in zip(previous[body], orientation)) < 0:
                        orientation = tuple(-v for v in orientation)
                    previous[body] = orientation
                    writer.writerow((run["id"], run["status"], cycle+1, elapsed, elapsed-start,
                                     stamp, frame, body, *position, *orientation, TENDONS[index],
                                     control_time, active_load["setpoint"], active_load["unit"],
                                     active_load["load_kgf"],
                                     *(loads.get(t, {}).get("load_kgf", "") for t in TENDONS)))
        results.append((str(destination.relative_to(output)), run["id"], run["status"],
                        TENDONS[index], cycle+1, end-start, len(samples)))
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, default=Path("output/experiment"))
    parser.add_argument("--output-dir", type=Path, default=Path("output_postprocessed"))
    args = parser.parse_args()
    if not args.input_dir.is_dir():
        parser.error(f"Recording directory does not exist: {args.input_dir}")
    results = []
    seen = set()
    for path in sorted(args.input_dir.rglob("run.json")):
        if not path.parent.parent.name.startswith("exp1_"):
            continue
        if not all(path.with_name(name).is_file() for name in ("controls.csv", "poses.csv")):
            continue
        run = json.loads(path.read_text())
        if run["simulated"] or run["status"] not in ("complete", "failed", "cancelled", "interrupted"):
            continue
        if run["id"] in seen:
            continue
        results.extend(export_run(path, run, args.output_dir))
        seen.add(run["id"])
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with (args.output_dir / "manifest.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("csv", "run_id", "run_status", "tendon", "cycle", "duration_s", "paired_frames"))
        writer.writerows(results)
    print(f"Exported {len(results)} complete cycles from {len({row[1] for row in results})} trials.")


if __name__ == "__main__":
    main()
