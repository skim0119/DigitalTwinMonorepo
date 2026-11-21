"""Jog one Spirob motor by a fixed number of STEP pulses."""

import argparse
import json
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAX_RPM = 30
STEPS_PER_REV = 1600


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

    from control_core import open_manual_controller

    forward = args.direction in ("forward", "1")
    rpm = args.speed * MAX_RPM
    rate = rpm * STEPS_PER_REV / 60
    sign = 1 if forward else -1
    target = sign * args.steps
    mode = "Simulated GPIO" if args.dry_run else "Hardware GPIO"
    print(f"{mode} | Motor {args.motor} | {'forward' if forward else 'backward'}", flush=True)
    print(f"  BCM GPIO: STEP {pul}, DIR {direction_pin}", flush=True)
    print(f"  Target: {args.steps} pulses at {rpm:g} RPM ({args.speed:.0%} of {MAX_RPM} RPM)",
          flush=True)

    motors = open_manual_controller(ROOT / "pins.json", motor=args.motor, simulate=args.dry_run)
    started = time.monotonic()
    result = "Complete"
    try:
        while motors.manual_command(args.motor, sign * rate, target) != target:
            time.sleep(0.01)
    except KeyboardInterrupt:
        result = "Interrupted"
    finally:
        motors.stop()
    count = abs(motors.status()[args.motor][1])
    print(f"{result} | {count}/{args.steps} STEP pulses in {time.monotonic() - started:.2f} s")


if __name__ == "__main__":
    main()
