"""Test one motor's force PID using the HX711 streamer."""

import argparse
import json
import math
from itertools import groupby
from pathlib import Path
from statistics import fmean


BASE_TARGET_KG = 0.1


def nonnegative(value):
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise argparse.ArgumentTypeError("expected a finite number at or above zero")
    return number


def report_performance(channel, schedule, hold_s):
    print(f"\nPID performance for {channel.name} (Kp={channel.kp:g}, Ki={channel.ki:g}, Kd={channel.kd:g})")
    print(f"  Tightening STEP pulses: {channel.tightening_steps}")
    if not channel.log_times:
        print("No force samples recorded.")
        return

    samples = zip(channel.log_times, channel.log_measured)
    for index, hold in groupby(samples, key=lambda sample: min(int(sample[0] / hold_s), len(schedule) - 1)):
        times, readings = zip(*hold)
        previous = schedule[index - 1] if index else readings[0]
        print(f"\n  Hold {index + 1}/{len(schedule)}")
        report_hold([t - times[0] for t in times], readings, schedule[index], previous)


def report_hold(times, readings, target, previous):
    target_g = target * 1000
    readings_g = [force * 1000 for force in readings]
    errors_g = [force - target_g for force in readings_g]
    rmse_g = math.sqrt(fmean(error ** 2 for error in errors_g))
    tail_start = times[0] + 0.8 * (times[-1] - times[0])
    tail_error_g = fmean(error for t, error in zip(times, errors_g) if t >= tail_start)
    rising = target >= previous
    peak_g = max(readings_g) if rising else min(readings_g)
    overshoot_g = max(0.0, peak_g - target_g if rising else target_g - peak_g)

    tolerance_g = max(2.0, 0.05 * target_g)
    last_outside = max((i for i, error in enumerate(errors_g) if abs(error) > tolerance_g), default=-1)
    settled_at = last_outside + 1
    if settled_at < len(times) and times[-1] - times[settled_at] >= 1.0:
        settling = f"{times[settled_at]:.2f} s"
    else:
        settling = "not observed"

    print(f"  Target {target_g:.1f} g | {len(times)} samples through {times[-1]:.2f} s")
    print(f"  Final {readings_g[-1]:.1f} g | {'peak' if rising else 'minimum'} {peak_g:.1f} g"
          f" | {'overshoot' if rising else 'undershoot'} {overshoot_g:.1f} g")
    print(f"  Tracking RMSE {rmse_g:.1f} g | mean error over final 20% {tail_error_g:+.1f} g")
    print(f"  Settling within ±{tolerance_g:g} g through last sample (at least 1 s): {settling}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motor", type=int, choices=range(6), required=True)
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[1] / "pins.json",
                        help="Use an alternate pins.json-format file.")
    parser.add_argument("--target-kg", type=nonnegative, action="append", required=True,
                        help="Target tension in kg. Repeat to test targets in order.")
    for gain in ("kp", "ki", "kd"):
        parser.add_argument(f"--{gain}", type=nonnegative,
                            help="Override the selected motor's gain in pins.json.")
    parser.add_argument("--seconds", type=nonnegative, default=3.0,
                        help="Hold time per target in seconds (default: %(default)g).")
    parser.add_argument("--deadband-g", type=nonnegative,
                        help="Force error needed to restart motion (defaults to shared PID setting).")
    parser.add_argument("--max-tension-kg", type=nonnegative,
                        help="Stop the trial at this measured tension.")
    parser.add_argument("--max-tightening-steps", type=int,
                        help="Stop the trial after this many tightening STEP pulses.")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be greater than zero")

    schedule = args.target_kg.copy()
    if schedule[0] != BASE_TARGET_KG:
        schedule.insert(0, BASE_TARGET_KG)
    release = schedule[-1] > BASE_TARGET_KG
    if release:
        schedule.append(BASE_TARGET_KG)
    duration = args.seconds * len(schedule)
    if not math.isfinite(duration):
        parser.error("Total trial duration must be finite")
    if args.max_tension_kg is not None and args.max_tension_kg <= max(schedule):
        parser.error("--max-tension-kg must exceed every target, including the 0.1 kg baseline")
    if args.max_tightening_steps is not None and args.max_tightening_steps <= 0:
        parser.error("--max-tightening-steps must be greater than zero")

    gains = {gain: value for gain in ("kp", "ki", "kd") if (value := getattr(args, gain)) is not None}
    from PID_multi import main as run_pid

    print(f"Baseline hold: {BASE_TARGET_KG:g} kg for {args.seconds:g} s.")
    if release:
        print(f"Release hold after the trial: {BASE_TARGET_KG:g} kg for {args.seconds:g} s.")
    channels = run_pid(
        args.motor,
        tuning=dict(
            **gains, setpoint_schedule=schedule,
            deadband_g=args.deadband_g,
            max_tension_kg=args.max_tension_kg, max_tightening_steps=args.max_tightening_steps,
        ),
        duration_s=duration,
        config=args.config,
    )
    channel = channels[0]
    report_performance(channel, schedule, args.seconds)
    if channel.log_times:
        gains = {"kp": channel.kp, "ki": channel.ki, "kd": channel.kd}
        print(f"\nIf you accept this trial, replace the PID entry for motor {args.motor} in pins.json:")
        print('"pid": ' + json.dumps(gains, allow_nan=False))


if __name__ == "__main__":
    main()
