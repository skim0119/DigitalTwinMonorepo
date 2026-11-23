"""Run an Exp 2 force staircase through Experiment API."""

import argparse

if __package__:
    from .experiment_client import add_api_arguments, parse_api_arguments, positive_float, run_trial
    from .quasi_static import make_pair_table
else:
    from experiment_client import add_api_arguments, parse_api_arguments, positive_float, run_trial
    from quasi_static import make_pair_table


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    add_api_arguments(parser)
    parser.add_argument("-F", "--force-kgf", type=positive_float, default=2.5)
    parser.add_argument("-C", "--cycles", type=int, choices=range(1, 31), default=1)
    increments = parser.add_mutually_exclusive_group()
    increments.add_argument("--step-kgf", type=positive_float, default=0.2)
    increments.add_argument("--steps", type=int, choices=range(1, 101))
    parser.add_argument("--dwell-s", type=positive_float, default=1.0)
    args = parse_api_arguments(parser, argv)

    try:
        raw, cycle_duration = make_pair_table(
            args.force_kgf, args.cycles, args.steps, args.dwell_s,
            step_kgf=args.step_kgf,
        )
    except ValueError as error:
        parser.error(str(error))
    return run_trial(
        args, raw, cycle_duration * args.cycles, label="Exp 2 bending",
        folder_prefix="exp2_api_", mode="force", unit="kgf", required_body_prefix="arm_",
    )


if __name__ == "__main__":
    raise SystemExit(main())
