"""CSV input validation for tendon commands."""
from __future__ import annotations

import csv
import io

import numpy as np


class ActuationRangeError(ValueError):
    """A tendon command falls outside the allowed range."""


def parse_actuation(content, count, maximum):
    """Read time_s and tendon_1..tendon_N, in N or mm."""
    if len(content) > 1_000_000:
        raise ValueError("CSV must be at most 1 MB")
    try:
        reader = csv.reader(io.StringIO(content.decode("utf-8-sig")))
        header = next(reader)
        expected = ["time_s"] + [f"tendon_{i + 1}" for i in range(count)]
        if [name.strip().lower() for name in header] != expected:
            raise ValueError("Expected header: " + ",".join(expected))
        rows = []
        for row in reader:
            if not row or all(not cell.strip() for cell in row):
                continue
            if len(row) != count + 1:
                raise ValueError(f"Each row needs {count + 1} columns")
            rows.append([float(cell) for cell in row])
            if len(rows) > 2000:
                raise ValueError("CSV may contain at most 2000 data rows")
    except (UnicodeDecodeError, StopIteration, csv.Error) as exc:
        raise ValueError("CSV must be UTF-8 with a header and numeric rows") from exc
    except ValueError as exc:
        if str(exc).startswith("could not convert"):
            raise ValueError("All CSV values must be numeric") from exc
        raise
    data = np.asarray(rows, dtype=float)
    if len(rows) < 2:
        raise ValueError("Provide at least two data rows")
    if not np.isfinite(data).all():
        raise ValueError("All CSV values must be finite")
    if data[0, 0] != 0 or np.any(np.diff(data[:, 0]) <= 0):
        raise ValueError("time_s must start at 0 and increase strictly")
    if np.any(data[:, 1:] < 0) or np.any(data[:, 1:] > maximum):
        raise ActuationRangeError(f"Tendon values must be between 0 and {maximum:g}")
    return data


def parse_experiment(content, count, maximum=50):
    """Read measured tendon loads from a postprocessed experimental cycle."""
    if count != 6:
        raise ValueError("Experimental cycles require six tendons")
    try:
        reader = csv.DictReader(io.StringIO(content.decode("utf-8-sig")))
        required = {"body", "cycle_elapsed_s", *(f"tendon_{i}_load_kgf" for i in range(1, 7))}
        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise ValueError("Expected an experimental cycle CSV with six measured tendon loads")
        rows = []
        for row in reader:
            if row["body"] != "arm_base":
                continue
            sample = [float(row["cycle_elapsed_s"])] + [
                9.80665 * float(row[f"tendon_{i}_load_kgf"]) for i in range(1, 7)
            ]
            if rows and sample[0] == rows[-1][0]:
                if sample != rows[-1]:
                    raise ValueError("Conflicting loads at the same cycle time")
                continue
            rows.append(sample)
    except (UnicodeDecodeError, csv.Error, TypeError, KeyError) as exc:
        raise ValueError("Invalid experimental cycle CSV") from exc
    if len(rows) < 2:
        raise ValueError("Experimental cycle must have at least two arm_base samples")
    data = np.asarray(rows)
    if not np.isfinite(data).all() or data[0, 0] < 0 or np.any(np.diff(data[:, 0]) <= 0):
        raise ValueError("Experimental cycle times and loads must be finite, with increasing nonnegative times")
    if maximum is not None and (np.any(data[:, 1:] < 0) or np.any(data[:, 1:] > maximum)):
        raise ValueError(f"Experimental measured loads outside 0–{maximum:g} N")
    if data[0, 0] > 0:
        data = np.vstack(([0.0, *data[0, 1:]], data))
    return data
