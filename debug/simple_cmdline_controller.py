"""Jog one Spirob motor by a fixed number of STEP pulses."""

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAX_RPM = 30
STEPS_PER_REV = 1600
PULSE_WIDTH_S = 10e-6


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("motor", type=int, choices=range(6), help="Motor number in pins.json.")
    parser.add_argument("direction", choices=("forward", "backward", "1", "-1"))
    parser.add_argument("speed", type=float, nargs="?", default=0.1,
                        help="Fraction of 30 RPM, from 0.01 to 1 (default: 0.1).")
    parser.add_argument("--steps", type=int, default=100, help="STEP pulses to send (default: 100).")
    parser.add_argument("--dry-run", "--debug", action="store_true",
                        help="Use simulated GPIO and print the same run summary.")
    args = parser.parse_args()
    if not 0.01 <= args.speed <= 1:
        parser.error("speed must be between 0.01 and 1")
    if args.steps <= 0:
        parser.error("--steps must be greater than zero")

    try:
        pins = json.loads((ROOT / "pins.json").read_text())["motors"][args.motor]
        pul, direction_pin = pins["pul"], pins["dir"]
        if (pins["motor"] != args.motor or pul == direction_pin
                or not all(type(pin) is int and 0 <= pin < 28 for pin in (pul, direction_pin))):
            raise ValueError("invalid motor number or BCM GPIO assignments")
    except (OSError, ValueError, KeyError, IndexError, TypeError) as error:
        parser.error(f"could not read motor {args.motor} from pins.json: {error}")

    sys.path.insert(0, str(ROOT / "Spirob_Code/core_nodes/motor"))
    from motors import Motors

    forward = args.direction in ("forward", "1")
    rpm = args.speed * MAX_RPM
    period = 60 / (rpm * STEPS_PER_REV)
    mode = "Simulated GPIO" if args.dry_run else "Hardware GPIO"
    print(f"{mode} | Motor {args.motor} | {'forward' if forward else 'backward'}", flush=True)
    print(f"  BCM GPIO: STEP {pul}, DIR {direction_pin}", flush=True)
    print(f"  Target: {args.steps} pulses at {rpm:g} RPM ({args.speed:.0%} of {MAX_RPM} RPM)",
          flush=True)

    motors = Motors([pul], [direction_pin], [True], steps_per_rev=STEPS_PER_REV,
                    max_rpm=MAX_RPM, pulse_width=PULSE_WIDTH_S, debug=args.dry_run)
    started = time.monotonic()
    result = "Complete"
    try:
        (motors.dir_motor_forward if forward else motors.dir_motor_backward)(0)
        time.sleep(PULSE_WIDTH_S)
        motors.set_motor_speed(0, args.speed)
        motors.start_motor(0)
        for _ in range(args.steps):
            motors.update(0, wait=False)
            time.sleep(PULSE_WIDTH_S)
            motors.update(0, wait=False)
            time.sleep(period - PULSE_WIDTH_S)
    except KeyboardInterrupt:
        result = "Interrupted"
    finally:
        try:
            motors._pul_motor_off(0)
        finally:
            motors.close()

    count = abs(motors.num_of_steps[0])
    print(f"{result} | {count}/{args.steps} STEP pulses in {time.monotonic() - started:.2f} s")


if __name__ == "__main__":
    main()
