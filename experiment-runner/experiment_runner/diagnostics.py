"""Export a quick-look plot from a trial's recorded CSVs."""
import csv
from bisect import bisect_left
from collections import defaultdict

from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib.lines import Line2D

from .table import TENDONS

POSTURE = (
    ("X (m)", "x_m"), ("Y (m)", "y_m"), ("Z (m)", "z_m"),
    ("Quaternion qx", "qx"), ("Quaternion qy", "qy"),
    ("Quaternion qz", "qz"), ("Quaternion qw", "qw"),
)
DIRECTIONS = ((1, "^", "Loading"), (-1, "v", "Unloading"), (0, "o", "Hold"))


def read_series(path, group_field):
    series = defaultdict(list)
    if path.is_file():
        with path.open(newline="") as stream:
            for row in csv.DictReader(stream):
                series[row[group_field]].append(row)
    return series


def hold_end_samples(series, times, cutoff):
    """Take the last sample inside each completed hold, keyed by its end time."""
    sampled = {}
    for name, rows in series.items():
        sample_times = [float(row["elapsed_s"]) for row in rows]
        endpoints = []
        for start, end in zip(times, times[1:]):
            if end > cutoff:
                break
            index = bisect_left(sample_times, end) - 1
            if index >= 0 and sample_times[index] >= start:
                endpoints.append(dict(rows[index], elapsed_s=end))
        if endpoints:
            sampled[name] = endpoints
    return sampled


def save_diagnostics(folder, table=None, *, complete=False):
    poses = read_series(folder / "poses.csv", "body")
    controls = read_series(folder / "controls.csv", "tendon")
    if not (poses or controls):
        return

    plot(folder / "diagnostics.png", poses, controls)
    if table is not None and table.mode == "force":
        cutoff = table.duration if complete else max(
            float(rows[-1]["elapsed_s"]) for series in (poses, controls) for rows in series.values()
        )
        hold_poses = hold_end_samples(poses, table.times, cutoff)
        hold_controls = hold_end_samples(controls, table.times, cutoff)
        if hold_poses or hold_controls:
            plot(folder / "diagnostics_step_end.png", hold_poses, hold_controls, step_end=True)
            plot_hysteresis(folder / "diagnostics_hysteresis.png", table, hold_poses, hold_controls)


def plot(path, poses, controls, *, step_end=False):
    units = next(iter(controls.values()))[0]["unit"] if controls else ""
    panels = [
        *((label, field, poses) for label, field in POSTURE),
        (f"Command ({units})" if units else "Command", "setpoint", controls),
        ("Load (kgf)", "load_kgf", controls),
    ]
    figure = Figure(figsize=(10, 1.7 * len(panels)), layout="constrained")
    axes = figure.subplots(len(panels), sharex=True)
    for axis, (label, field, series) in zip(axes, panels):
        for name, rows in series.items():
            times = [float(row["elapsed_s"]) for row in rows]
            values = [float(row[field]) if row[field] else float("nan") for row in rows]
            axis.plot(times, values, label=name, marker="." if step_end or len(times) == 1 else None)
        axis.set_ylabel(label)
        axis.grid(alpha=0.25)
        if not series:
            axis.text(0.5, 0.5, "No samples", transform=axis.transAxes, ha="center")
        elif field in ("x_m", "setpoint"):
            axis.legend(loc="best", ncols=3, fontsize="small")
    axes[-1].set_xlabel("Hold end (s)" if step_end else "Elapsed time (s)")
    save_png(figure, path)


def plot_hysteresis(path, table, poses, controls):
    pose_by_hold = {
        name: {row["elapsed_s"]: row for row in rows} for name, rows in poses.items()
    }
    columns = {}
    for index, tendon in enumerate(TENDONS):
        commands = [row[index] for row in table.values[:-1]]
        if len(set(commands) - {None}) < 2:
            continue
        directions = {
            end: (value > previous) - (value < previous)
            for end, value, previous in zip(table.times[1:], commands, commands[:1] + commands)
            if value is not None and previous is not None
        }
        samples = [(row["elapsed_s"], float(row["load_kgf"]))
                   for row in controls.get(tendon, ()) if row["load_kgf"]]
        series = {}
        for name, rows in pose_by_hold.items():
            matched = [(force, directions[end], rows[end]) for end, force in samples if end in rows and end in directions]
            if matched:
                series[name] = matched
        if series:
            columns[tendon] = series
    if not columns:
        return

    figure = Figure(figsize=(5 * len(columns) + 1, 1.7 * len(POSTURE)), layout="constrained")
    axes = figure.subplots(len(POSTURE), len(columns), sharex="col", squeeze=False)
    for (tendon, series), column in zip(columns.items(), axes.T):
        column[0].set_title(tendon)
        for name, points in series.items():
            forces, directions, rows = zip(*points)
            values = [[float(row[field]) for _, field in POSTURE] for row in rows]
            # q and -q describe the same posture. Keep consecutive signs aligned.
            for previous, current in zip(values, values[1:]):
                if sum(a * b for a, b in zip(previous[3:], current[3:])) < 0:
                    current[3:] = [-value for value in current[3:]]
            for axis, (label, _), component in zip(column, POSTURE, zip(*values)):
                line, = axis.plot(forces, component, label=name, linewidth=1, alpha=.65)
                for direction, marker, _ in DIRECTIONS:
                    indices = [i for i, phase in enumerate(directions) if phase == direction]
                    axis.scatter([forces[i] for i in indices], [component[i] for i in indices],
                                 marker=marker, color=line.get_color(), s=18)
                axis.set_ylabel(label)
                axis.grid(alpha=.25)
        column[0].legend(loc="best", fontsize="small")
        column[-1].set_xlabel("Measured tendon force (kgf)")
    figure.legend(
        handles=[Line2D([], [], marker=marker, linestyle="none", color="0.3", label=label)
                 for _, marker, label in DIRECTIONS],
        loc="outside upper center", ncols=len(DIRECTIONS),
    )
    save_png(figure, path)


def save_png(figure, path):
    canvas = FigureCanvasAgg(figure)
    pending = path.with_suffix(".tmp")
    try:
        canvas.print_png(pending)
        pending.replace(path)
    finally:
        pending.unlink(missing_ok=True)
