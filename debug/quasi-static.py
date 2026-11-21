"""Measure the force response to a staircase on one Spirob motor."""

import argparse
import math
from pathlib import Path

from PID_multi import DEFAULT_CONFIG, main as run_pid
from PID_tune_motor import BASE_TARGET_KG, nonnegative, report_performance


def staircase(peak, increment):
    rising = [BASE_TARGET_KG]
    for index in range(1, math.ceil((peak - BASE_TARGET_KG) / increment)):
        level = BASE_TARGET_KG + index * increment
        if math.isclose(level, peak, abs_tol=0, rel_tol=1e-12):
            break
        rising.append(level)
    return [*rising, peak, *reversed(rising)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motor", type=int, choices=range(6), required=True)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--target-kg", type=nonnegative, required=True, help="Peak tension in kgf.")
    parser.add_argument("--step-kg", type=nonnegative, required=True, help="Force increment in kgf.")
    parser.add_argument("-S", "--step-s", type=nonnegative, default=2.0,
                        help="Hold time per setpoint in seconds (default: %(default)g).")
    parser.add_argument("--max-tension-kg", type=nonnegative, required=True)
    parser.add_argument("--max-tightening-steps", type=int, required=True)
    args = parser.parse_args()

    if not BASE_TARGET_KG < args.target_kg < args.max_tension_kg:
        parser.error(f"Peak force must be above {BASE_TARGET_KG:g} kgf and below the tension limit.")
    if not (args.step_kg > 0 and args.step_s > 0 and args.max_tightening_steps > 0):
        parser.error("Use positive force increments, hold time, and tightening pulse limit.")
    increments = (args.target_kg - BASE_TARGET_KG) / args.step_kg
    if not 0 <= increments <= 1000:
        parser.error("Use a larger step size, with at most 1,000 increments to the peak.")
    if not math.isfinite((2 * math.ceil(increments) + 1) * args.step_s):
        parser.error("Total trial duration must be finite.")

    schedule = staircase(args.target_kg, args.step_kg)
    print(f"Motor {args.motor} | {len(schedule)} holds of {args.step_s:g} s | "
          f"{BASE_TARGET_KG:g} → {args.target_kg:g} → {BASE_TARGET_KG:g} kgf")
    channel, = run_pid(
        args.motor, config=args.config, duration_s=len(schedule) * args.step_s,
        tuning=dict(
            setpoint_schedule=schedule,
            max_tension_kg=args.max_tension_kg,
            max_tightening_steps=args.max_tightening_steps,
        ),
    )
    report_performance(channel, schedule, args.step_s)


if __name__ == "__main__":
    main()
