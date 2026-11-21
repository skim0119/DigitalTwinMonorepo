"""Build force staircases for the Experiment API's zero-order hold playback."""

import csv
import io
import math
from fractions import Fraction

TENDONS = tuple(f"tendon_{index + 1}" for index in range(6))
PRETENSION_KGF = 0.1


def force_staircase(tendons, waypoints, cycles, steps, dwell, *, step_kgf=0.2, pretension_kgf=PRETENSION_KGF):
    """Step between active-tendon force waypoints in kgf and repeat the cycle."""
    step_size = Fraction(str(step_kgf))
    phases = []
    for start, end in zip(waypoints, waypoints[1:]):
        start = tuple(Fraction(str(value)) for value in start)
        end = tuple(Fraction(str(value)) for value in end)
        distance = max(abs(b - a) for a, b in zip(start, end))
        count = steps if steps is not None else max(1, math.ceil(distance / step_size))
        phases.append((start, end, distance, count))

    level_count = 1 + sum(count for _, _, _, count in phases)
    ticks = level_count * cycles
    if ticks * dwell > 600:
        raise ValueError("Total duration must be at most 600 s.")
    if ticks > 256 * 1024 // 14:
        raise ValueError("The force table would exceed the API upload limit.")

    levels = [waypoints[0]]
    for start, end, distance, count in phases:
        for step in range(1, count + 1):
            if steps is not None or distance == 0:
                progress = Fraction(step, count)
            else:
                progress = min(1, step * step_size / distance)
            levels.append(tuple(float(a + (b - a) * progress) for a, b in zip(start, end)))

    stream = io.StringIO(newline="")
    writer = csv.writer(stream)
    writer.writerow(("time_s", *TENDONS))
    for tick, level in enumerate(levels * cycles + [levels[-1]]):
        values = [pretension_kgf] * len(TENDONS)
        for tendon, force in zip(tendons, level):
            values[tendon] = force
        writer.writerow((tick * dwell, *values))
    raw = stream.getvalue().encode()
    if len(raw) > 256 * 1024:
        raise ValueError("The force table exceeds the API upload limit.")
    return raw, level_count * dwell
