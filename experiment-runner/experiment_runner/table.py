"""The actuation table contract shared by validation and playback."""
import csv
import io
import math
from bisect import bisect_right
from dataclasses import dataclass

TENDONS = tuple(f"tendon_{i}" for i in range(1, 7))
MAX_CSV = 256 * 1024
UNITS = {"force": {"N", "kgf"}, "displacement": {"mm"}}


@dataclass(frozen=True)
class Table:
    mode: str
    unit: str
    times: tuple
    values: tuple

    @property
    def duration(self):
        return self.times[-1]

    def at(self, elapsed):
        index = min(len(self.times) - 1, bisect_right(self.times, elapsed) - 1)
        index = max(index, 0)
        if self.mode == "force" or index == len(self.times) - 1:
            return self.values[index]
        ratio = (max(0, elapsed) - self.times[index]) / (self.times[index + 1] - self.times[index])
        return tuple(a * (1 - ratio) + b * ratio for a, b in zip(self.values[index], self.values[index + 1]))

    def response(self):
        return dict(mode=self.mode, unit=self.unit, duration=self.duration,
                    times=self.times, values=self.values,
                    interpolation="hold" if self.mode == "force" else "linear")


def parse_table(raw, mode, unit, *, optional_tendons=False):
    if unit not in UNITS.get(mode, set()):
        raise ValueError("Choose force in N or kgf, or displacement in mm.")
    if not raw or len(raw) > MAX_CSV:
        raise ValueError("Upload a CSV smaller than 256 KiB.")
    try:
        reader = csv.reader(io.StringIO(raw.decode("utf-8-sig"), newline=""), strict=True)
        header = next(reader, [])
        sparse = optional_tendons and mode == "force"
        columns = header[1:]
        expected = [name for name in TENDONS if name in columns] if sparse else list(TENDONS)
        if not columns or header != ["time_s", *expected]:
            raise ValueError("Expected time_s followed by tendon columns in tendon_1 through tendon_6 order.")
        times, values = [], []
        for line, row in enumerate(reader, 2):
            if len(row) != len(header):
                raise ValueError(f"Row {line} must match the CSV header.")
            try:
                stamp = float(row[0])
                commands = {name: float(cell) for name, cell in zip(columns, row[1:])
                            if not sparse or cell.strip()}
            except ValueError:
                raise ValueError(f"Row {line} contains an invalid number.") from None
            if not math.isfinite(stamp) or not all(map(math.isfinite, commands.values())):
                raise ValueError(f"Row {line} contains a non-finite number.")
            tendons = tuple(commands.get(name) for name in TENDONS)
            if (not times and stamp != 0) or (times and stamp <= times[-1]) or stamp > 600:
                raise ValueError(f"Row {line}: time must start at zero, increase, and stay within 600 s.")
            if mode == "force" and any(x < 0 for x in commands.values()):
                raise ValueError(f"Row {line}: force cannot be negative.")
            times.append(stamp)
            values.append(tuple(tendons))
    except (UnicodeError, csv.Error) as error:
        raise ValueError("The actuation table is not a valid UTF-8 CSV.") from error
    if len(times) < 2:
        raise ValueError("Include at least two rows.")
    return Table(mode, unit, tuple(times), tuple(values))
