"""Export a quick-look plot from a trial's recorded CSVs."""
import csv
from collections import defaultdict

from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure


def read_series(path, group_field):
    series = defaultdict(list)
    if path.is_file():
        with path.open(newline="") as stream:
            for row in csv.DictReader(stream):
                series[row[group_field]].append(row)
    return series


def save_diagnostics(folder):
    poses = read_series(folder / "poses.csv", "body")
    controls = read_series(folder / "controls.csv", "tendon")
    if not (poses or controls):
        return

    units = next(iter(controls.values()))[0]["unit"] if controls else ""
    panels = [
        ("X (m)", "x_m", poses),
        ("Y (m)", "y_m", poses),
        ("Z (m)", "z_m", poses),
        (f"Command ({units})" if units else "Command", "setpoint", controls),
        ("Load (kgf)", "load_kgf", controls),
    ]
    figure = Figure(figsize=(10, 9), layout="constrained")
    canvas = FigureCanvasAgg(figure)
    axes = figure.subplots(len(panels), sharex=True)
    for axis, (label, field, series) in zip(axes, panels):
        for name, rows in series.items():
            times = [float(row["elapsed_s"]) for row in rows]
            values = [float(row[field]) if row[field] else float("nan") for row in rows]
            axis.plot(times, values, label=name, marker="." if len(times) == 1 else None)
        axis.set_ylabel(label)
        axis.grid(alpha=0.25)
        if not series:
            axis.text(0.5, 0.5, "No samples", transform=axis.transAxes, ha="center")
    for index in (0, 3):
        if panels[index][2]:
            axes[index].legend(loc="best", ncols=3, fontsize="small")
    axes[-1].set_xlabel("Elapsed time (s)")
    pending = folder / "diagnostics.tmp"
    try:
        canvas.print_png(pending)
        pending.replace(folder / "diagnostics.png")
    finally:
        pending.unlink(missing_ok=True)
