"""Tune one motor's force PID using the HX711 streamer."""

import argparse
import math
from statistics import fmean


def nonnegative(value):
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise argparse.ArgumentTypeError("expected a finite number at or above zero")
    return number


def report_performance(channel):
    times = channel.log_times
    print(f"\nPID performance for {channel.name} (Kp={channel.kp:g}, Ki={channel.ki:g}, Kd={channel.kd:g})")
    print(f"  Tightening STEP pulses: {channel.tightening_steps}")
    if not times:
        print("No force samples recorded.")
        return

    target_g = channel.current_setpoint * 1000
    readings_g = [force * 1000 for force in channel.log_measured]
    errors_g = [force - target_g for force in readings_g]
    rmse_g = math.sqrt(fmean(error ** 2 for error in errors_g))
    tail_start = times[0] + 0.8 * (times[-1] - times[0])
    tail_error_g = fmean(error for t, error in zip(times, errors_g) if t >= tail_start)
    peak_g = max(readings_g)

    tolerance_g = max(2.0, 0.05 * target_g)
    last_outside = max((i for i, error in enumerate(errors_g) if abs(error) > tolerance_g), default=-1)
    settled_at = last_outside + 1
    if settled_at < len(times) and times[-1] - times[settled_at] >= 1.0:
        settling = f"{times[settled_at]:.2f} s"
    else:
        settling = "not observed"

    print(f"  Target {target_g:.1f} g | {len(times)} samples through {times[-1]:.2f} s")
    print(f"  Final {readings_g[-1]:.1f} g | peak {peak_g:.1f} g | overshoot {max(0.0, peak_g - target_g):.1f} g")
    print(f"  Tracking RMSE {rmse_g:.1f} g | mean error over final 20% {tail_error_g:+.1f} g")
    print(f"  Settling within ±{tolerance_g:g} g through trial end (at least 1 s): {settling}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motor", type=int, choices=range(6), required=True)
    parser.add_argument("--target-kg", type=nonnegative, required=True)
    parser.add_argument("--kp", type=nonnegative, help="Defaults to the selected motor’s gain in PID_multi.py.")
    parser.add_argument("--ki", type=nonnegative, default=0.0)
    parser.add_argument("--kd", type=nonnegative, default=0.0)
    parser.add_argument("--seconds", type=nonnegative, default=3.0,
                        help="Trial duration in seconds (default: %(default)g).")
    parser.add_argument("--deadband-g", type=nonnegative,
                        help="Force error needed to restart motion (defaults to shared PID setting).")
    parser.add_argument("--max-tension-kg", type=nonnegative,
                        help="Stop the trial at this measured tension.")
    parser.add_argument("--max-tightening-steps", type=int,
                        help="Stop the trial after this many tightening STEP pulses.")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be greater than zero")

    if args.max_tension_kg is not None and args.max_tension_kg <= args.target_kg:
        parser.error("--max-tension-kg must exceed --target-kg")
    if args.max_tightening_steps is not None and args.max_tightening_steps <= 0:
        parser.error("--max-tightening-steps must be greater than zero")

    from PID_multi import CHANNEL_CONFIGS, FORCE_DEADBAND_G, main as run_pid

    kp = CHANNEL_CONFIGS[args.motor]["kp"] if args.kp is None else args.kp

    channels = run_pid(
        args.motor,
        tuning=dict(
            kp=kp, ki=args.ki, kd=args.kd, setpoint_schedule=[args.target_kg],
            deadband_g=FORCE_DEADBAND_G if args.deadband_g is None else args.deadband_g,
            max_tension_kg=args.max_tension_kg, max_tightening_steps=args.max_tightening_steps,
        ),
        duration_s=args.seconds,
    )
    report_performance(channels[0])


if __name__ == "__main__":
    main()
