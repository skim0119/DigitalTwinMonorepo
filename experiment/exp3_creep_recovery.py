"""Measure single-tendon creep and recovery with fixed 10-second loading and unloading legs."""

import argparse
import math
import sys
from fractions import Fraction

if __package__:
    from .collection import add_collection_arguments, collect_trial, validate_collection_arguments
    from .quasi_static import PRETENSION_KGF, TENDONS, force_table
else:
    from collection import add_collection_arguments, collect_trial, validate_collection_arguments
    from quasi_static import PRETENSION_KGF, TENDONS, force_table

LEG_S = 10
STEPS_PER_LEG = 25


def make_creep_table(tendon, force, baseline_s, hold_s, recovery_s):
    """Build a 0.1 kgf baseline, timed loading, peak hold, and recovery."""
    if not math.isfinite(force) or force <= PRETENSION_KGF:
        raise ValueError(f"The peak force must exceed the {PRETENSION_KGF:g} kgf preload.")
    times = (baseline_s, hold_s, recovery_s)
    if not all(math.isfinite(value) and value > 0 for value in times):
        raise ValueError("Baseline, hold, and recovery times must be finite and positive.")

    baseline, hold, recovery = (Fraction(str(value)) for value in times)
    peak_start = baseline + LEG_S
    unloading_start = peak_start + hold
    recovery_start = unloading_start + LEG_S
    end = recovery_start + recovery
    if end > 600:
        raise ValueError("Total duration must be at most 600 s.")

    preload = Fraction(str(PRETENSION_KGF))
    peak = Fraction(str(force))
    tick = Fraction(LEG_S, STEPS_PER_LEG)
    increment = (peak - preload) / STEPS_PER_LEG
    half_step = increment / 2
    levels = [(0, preload)]
    # Midpoint setpoints keep the two ramps symmetric and the peak hold exact.
    levels.extend((baseline + i * tick, preload + half_step + i * increment)
                  for i in range(STEPS_PER_LEG))
    levels.append((peak_start, peak))
    levels.extend((unloading_start + i * tick, peak - half_step - i * increment)
                  for i in range(STEPS_PER_LEG))
    levels.extend(((recovery_start, preload), (end, preload)))
    timed_levels = ((float(t), (float(value),)) for t, value in levels)
    return force_table((tendon,), timed_levels), float(end)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.set_defaults(single_mode=False)
    parser.add_argument("-N", "--tendon", type=int, choices=range(6), default=0,
                        help="zero-based motor index (default: 0)")
    parser.add_argument("-F", "--force-kgf", type=float, default=1.0,
                        help="peak tendon force in kgf (default: 1.0)")
    parser.add_argument("--baseline-s", type=float, default=30, help="baseline at preload (default: 30 s)")
    parser.add_argument("--hold-s", type=float, default=180, help="peak hold (default: 180 s)")
    parser.add_argument("--recovery-s", type=float, default=300, help="recovery at preload (default: 300 s)")
    add_collection_arguments(parser)
    args = parser.parse_args(argv)
    validate_collection_arguments(parser, args)
    try:
        raw, duration = make_creep_table(
            args.tendon, args.force_kgf, args.baseline_s, args.hold_s, args.recovery_s,
        )
    except ValueError as error:
        parser.error(str(error))

    tendon = TENDONS[args.tendon]
    if args.name is None:
        args.name = tendon
    return collect_trial(
        raw, (tendon,), duration, 1, args, number=3,
        label=f"Exp 3 creep and recovery ({tendon})", required_bodies=("arm_base", "arm_tip"),
    )


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RuntimeError, OSError, ValueError) as error:
        sys.exit(str(error))
