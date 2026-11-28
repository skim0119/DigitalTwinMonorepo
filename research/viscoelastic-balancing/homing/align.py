"""Home horizontal position and marker normals using paired counter-bend pulses."""
import sys
from pathlib import Path

import click
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from homing import BASE, DATA, HERE, MocapTimeout, SettleTimeout, run_on_hardware
from probe_model import rotation

PAIRS = ((5, 0), (3, 1), (4, 2))
GREEN = (0, 1, 0)
RELEASE_KGF = 0.05
FORCE_CUTOFF_KGF = 0.80
MAX_TARGET_KGF = 0.70
POSITION_TOL_MM = 5.0
MIN_GAIN = 0.10


def errors(pose):
    green = rotation(pose).apply(GREEN)
    return float(np.linalg.norm(pose[:2])), float(np.degrees(np.arccos(np.clip(abs(green[1]), 0, 1))))


class CounterBendPulse:
    """Track a filtered loss and stop a pull before it passes the local minimum."""

    def __init__(self, loss, initial, motor, peak):
        self.loss = loss
        self.initial = self.filtered = self.best = initial
        self.motor, self.peak = motor, peak
        self.start = self.loaded_at = self.improved_at = None
        self.reason = "settled"

    def __call__(self, pose, forces, now):
        if self.start is None:
            self.start = self.improved_at = now
        self.filtered += 0.25 * (self.loss(pose) - self.filtered)
        if self.filtered < self.best - 0.04:
            self.best, self.improved_at = self.filtered, now
        if self.loaded_at is None and forces[self.motor] >= BASE[self.motor] + 0.70 * (self.peak - BASE[self.motor]):
            self.loaded_at = now
        if now - self.start >= 0.4 and (self.filtered > self.initial + 0.25 or self.filtered > self.best + 0.20):
            self.reason = "loss rising"
        elif self.loaded_at is not None and now - max(self.loaded_at, self.improved_at) >= 0.55:
            self.reason = "loss plateau"
        elif now - self.start >= 2.4:
            self.reason = "pulse time limit"
        else:
            return False
        return True


def align(rig, *, max_tension, tolerance, max_trials, hold_seconds):
    def loss(pose):
        distance, tilt = errors(pose)
        position_error, tilt_error = distance / POSITION_TOL_MM, tilt / tolerance
        return max(position_error, tilt_error) + 0.1 * np.hypot(position_error, tilt_error)

    def inside(pose):
        distance, tilt = errors(pose)
        return distance <= POSITION_TOL_MM and tilt <= tolerance

    def near(pose):
        distance, tilt = errors(pose)
        return distance <= 1.15 * POSITION_TOL_MM and tilt <= 1.15 * tolerance

    def hold(targets, phase, *, duration=1.0, sample_stop=None, verify=False):
        return rig.hold(
            targets, phase, duration=duration, settle_timeout=35,
            axial_pose=True, local_normal=GREEN, pose_timeout=2.0,
            pose_check=inside if verify else None, sample_stop=sample_stop,
            decrement_rate=0.20 if phase == "return" else 0.10, settle_gap=0.15,
        )

    def report_pose(label, pose):
        distance, tilt = errors(pose)
        rig.report(f"{label} | horizontal error {distance:.2f} mm | green-axis error {tilt:.2f}°")

    def verify_baseline():
        pose, verified = hold(BASE, "return", duration=hold_seconds, verify=True)
        distance, tilt = errors(pose)
        report_pose("Verified baseline window", pose)
        if verified:
            rig.log(dict(kind="result", reason="Position and normal alignment verified at baseline",
                         position_mm=distance, tilt_deg=tilt, targets_kgf=BASE.tolist()))
        return pose, verified

    pose, _ = hold(BASE, "baseline")
    report_pose("Initial baseline", pose)
    if near(pose):
        pose, verified = verify_baseline()
        if verified:
            return
    directions = list(PAIRS)
    trial = 0
    try:
        for peak in (max_tension, max(0.15, BASE[0] + 0.7 * (max_tension - BASE[0]))):
            while trial < max_trials and not inside(pose):
                pass_start = loss(pose)
                for index, pair in enumerate(directions):
                    for motor, partner in (pair, pair[::-1]):
                        direction_helped = False
                        for _ in range(2):
                            if trial >= max_trials or inside(pose):
                                break
                            trial += 1
                            start_loss = loss(pose)
                            targets = BASE.copy()
                            fine = near(pose)
                            pulse_peak = min(peak, 0.30) if fine else peak
                            targets[motor], targets[partner] = pulse_peak, RELEASE_KGF
                            stop = CounterBendPulse(loss, start_loss, motor, pulse_peak)
                            rig.report(f"Pulse {trial}/{max_trials} | m{motor} {pulse_peak:.2f} kgf, m{partner} {RELEASE_KGF:.2f} kgf")
                            try:
                                loaded, _ = hold(targets, "pulse", sample_stop=stop)
                                report_pose(f"Release trigger ({stop.reason})", loaded)
                            except SettleTimeout as error:
                                rig.report(f"{error}. Returning to baseline.")
                            pose, _ = hold(BASE, "return")
                            report_pose(f"Released pulse {trial}", pose)
                            if near(pose):
                                pose, verified = verify_baseline()
                                if verified:
                                    return
                            gain = start_loss - loss(pose)
                            rig.log(dict(kind="pulse_result", trial=trial, motor=motor, partner=partner,
                                         stop=stop.reason, target_kgf=targets.tolist(),
                                         settled_pose=pose.tolist(), settled_loss=loss(pose), gain=gain))
                            if gain < MIN_GAIN:
                                break
                            directions[index] = (motor, partner)
                            direction_helped = True
                        if direction_helped:
                            break
                        if trial >= max_trials or inside(pose):
                            break
                    if trial >= max_trials or inside(pose):
                        break
                if pass_start - loss(pose) < MIN_GAIN:
                    break
    except (MocapTimeout, KeyboardInterrupt) as error:
        rig.report(f"{error or 'Interrupted'}. Returning to baseline.")
        pose, _ = hold(BASE, "return")
        report_pose("Released", pose)
        raise

    pose, verified = verify_baseline()
    if not verified:
        raise RuntimeError(f"Baseline restored after {trial} pulses. "
                           f"Required ≤{POSITION_TOL_MM:g} mm and ≤{tolerance:g}° throughout verification.")


@click.command()
@click.option("--data", type=click.Path(path_type=Path), default=DATA)
@click.option("--config", type=click.Path(path_type=Path), default=HERE.parents[1] / "pins.json")
@click.option("--max-tension", type=click.FloatRange(0.15, MAX_TARGET_KGF), default=0.65, show_default=True)
@click.option("--tolerance", type=click.FloatRange(0, 90, min_open=True, max_open=True),
              default=8.0, show_default=True, help="Green-axis tilt tolerance in degrees.")
@click.option("--max-trials", type=click.IntRange(1), default=24, show_default=True)
@click.option("--hold-seconds", type=click.IntRange(1, 30), default=3, show_default=True)
def main(data, config, max_tension, tolerance, max_trials, hold_seconds):
    """Align position and green marker axes, then verify at baseline tension."""
    if not np.isfinite([max_tension, tolerance]).all():
        raise click.UsageError("Tension and tolerance must be finite.")
    data.mkdir(parents=True, exist_ok=True)
    limits = (max_tension - BASE[0]) * np.eye(len(BASE))

    def run(rig):
        align(rig, max_tension=max_tension, tolerance=tolerance,
              max_trials=max_trials, hold_seconds=hold_seconds)

    run_on_hardware(run, actions=limits, mode="align", data=data, config=config, seed=0,
                    force_cap=FORCE_CUTOFF_KGF, world_translation=True)


if __name__ == "__main__":
    main()
