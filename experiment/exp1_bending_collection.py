"""Collect repeated quasi-static single-tendon force staircases on Spirob."""

import argparse
import math
import sys

if __package__:
    from .collection import add_collection_arguments, collect_trial, validate_collection_arguments
    from .quasi_static import TENDONS, make_pair_table, make_table
else:
    from collection import add_collection_arguments, collect_trial, validate_collection_arguments
    from quasi_static import TENDONS, make_pair_table, make_table


def main(*, pair=False):
    number = 2 if pair else 1
    parser = argparse.ArgumentParser(description=f"Collect quasi-static SpiRob bending data for Exp {number}.")
    parser.set_defaults(single_mode=False)
    if not pair:
        parser.add_argument("-N", "--tendon", type=int, choices=range(6), default=0)
        parser.add_argument("--single-mode", action="store_true",
                            help="apply force PID only to the selected tendon")
    parser.add_argument("-F", "--force-kgf", type=float, default=2.5)
    parser.add_argument("-C", "--cycles", type=int, choices=range(1, 31), default=1)
    increments = parser.add_mutually_exclusive_group()
    increments.add_argument("--step-kgf", type=float, default=0.2,
                            help="force increment in kgf (default: 0.2)")
    increments.add_argument("--steps", type=int, choices=range(1, 101),
                            help="use this many equal increments per phase")
    parser.add_argument("--dwell-s", type=float, default=1.0,
                        help="seconds to hold each setpoint (default: 1)")
    add_collection_arguments(parser)
    args = parser.parse_args()
    validate_collection_arguments(parser, args)
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
                step_kgf=args.step_kgf, single_mode=args.single_mode,
            )
            tendons = (TENDONS[args.tendon],)
    except ValueError as error:
        parser.error(str(error))

    label = f"Exp {number} bending collection"
    if args.single_mode:
        label += " (single mode)"
    return collect_trial(raw, tendons, cycle_duration, args.cycles, args, number=number, label=label)


def run_cli(*, pair=False):
    try:
        sys.exit(main(pair=pair))
    except (RuntimeError, OSError, ValueError) as error:
        sys.exit(str(error))


if __name__ == "__main__":
    run_cli()
