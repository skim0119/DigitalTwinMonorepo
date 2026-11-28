"""Align the tip marker's green axis with the base marker's green axis."""
import sys
from pathlib import Path

import click
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from homing import BASE, DATA, HERE, MocapTimeout, SettleTimeout, run_on_hardware
from probe_model import rotation

PAIRS = ((0, 5), (1, 3), (2, 4))
GREEN = (0, 1, 0)
RELEASE_KGF = 0.05
MIN_IMPROVEMENT_DEG = 0.75
FORCE_CUTOFF_KGF = 0.80
MAX_TARGET_KGF = 0.70
MAX_COMBINED_PULL_KGF = 0.80


def tilt(pose):
    green = rotation(pose).apply(GREEN)
    return float(np.degrees(np.arccos(np.clip(abs(green[1]), 0, 1))))


def tensions(biases):
    targets = BASE.copy()
    for (first, second), bias in zip(PAIRS, biases):
        if bias:
            pull, release = (first, second) if bias > 0 else (second, first)
            targets[pull] += abs(bias)
            targets[release] -= RELEASE_KGF
    return targets


class AlignmentNotReached(RuntimeError):
    """The bounded search did not verify green-axis alignment."""


def align(rig, *, max_tension, tolerance, max_trials, hold_seconds):
    def hold(biases, phase, *, duration=0.6, verify=False):
        targets = tensions(biases)
        envelope = np.maximum(rig.targets, targets)
        if np.maximum(envelope - BASE, 0).sum() > MAX_COMBINED_PULL_KGF + 1e-9:
            # Increases ramp faster than releases, so unload before transferring pull.
            rig.hold(np.minimum(rig.targets, targets), "transfer pull", settle_timeout=45,
                     axial_pose=True, local_normal=GREEN, pose_timeout=2.0)
        pose, verified = rig.hold(
            targets, phase, duration=duration, settle_timeout=45,
            axial_pose=True, local_normal=GREEN, pose_timeout=2.0,
            pose_check=(lambda sample: tilt(sample) <= tolerance) if verify else None,
        )
        angle = tilt(pose)
        rig.report(f"{phase} | green-axis error {angle:.2f}° | kgf "
                   + ", ".join(f"m{i}={value:.2f}" for i, value in enumerate(targets)))
        return angle, verified

    biases = np.zeros(len(PAIRS))
    angle, _ = hold(biases, "baseline")
    initial = angle
    best_biases, best_angle = biases.copy(), angle
    trials = 0
    limit = max_tension - float(BASE[0])
    for step in (0.25, 0.10, 0.05):
        improved = True
        while improved and trials < max_trials and angle > tolerance:
            improved = False
            for axis, pair in enumerate(PAIRS):
                if angle <= tolerance or trials >= max_trials:
                    break
                direction = -1 if axis == 0 or biases[axis] < 0 else 1
                if biases[axis] > 0:
                    direction = 1
                for sign in (direction, -direction):
                    while trials < max_trials and angle > tolerance:
                        candidate = biases.copy()
                        value = round(float(biases[axis]) + sign * step, 6)
                        other_pull = sum(abs(bias) for i, bias in enumerate(biases) if i != axis)
                        pair_limit = min(limit, max(0, MAX_COMBINED_PULL_KGF - other_pull))
                        candidate[axis] = np.clip(value, -pair_limit, pair_limit)
                        if candidate[axis] == biases[axis]:
                            break
                        trials += 1
                        phase = f"trial {trials} pair {pair[0]}/{pair[1]}"
                        try:
                            candidate_angle, _ = hold(candidate, phase)
                        except SettleTimeout as error:
                            rig.report(str(error))
                            angle, _ = hold(biases, "restore")
                            break
                        if candidate_angle < best_angle:
                            best_biases, best_angle = candidate.copy(), candidate_angle
                        if candidate_angle <= tolerance or angle - candidate_angle >= MIN_IMPROVEMENT_DEG:
                            biases, angle = candidate, candidate_angle
                            improved = True
                            continue
                        angle, _ = hold(biases, "restore")
                        break
                    if angle <= tolerance or trials >= max_trials:
                        break
        if angle <= tolerance or trials >= max_trials:
            break

    if best_angle < angle and not np.array_equal(best_biases, biases):
        biases = best_biases
        angle, _ = hold(biases, "best recorded targets")
    if angle <= tolerance:
        angle, verified = hold(biases, "verify", duration=hold_seconds, verify=True)
        if verified:
            rig.report(f"Aligned | {initial:.2f}° → {angle:.2f}° over {trials} trials")
            rig.log(dict(kind="loaded_alignment", tilt_deg=angle,
                         targets_kgf=tensions(biases).tolist()))
            return
    raise AlignmentNotReached(f"Green axes {initial:.2f}° → {angle:.2f}° after {trials} trials. "
                       f"Required ≤{tolerance:g}° throughout verification.")


def align_and_release(rig, *, max_tension, tolerance, max_trials, hold_seconds):
    interrupted = False
    try:
        align(rig, max_tension=max_tension, tolerance=tolerance,
              max_trials=max_trials, hold_seconds=hold_seconds)
    except (AlignmentNotReached, MocapTimeout, SettleTimeout) as error:
        rig.report(str(error))
    except KeyboardInterrupt:
        interrupted = True
        rig.report("Interrupted. Returning to baseline.")
    pose, verified = rig.hold(
        BASE, "return", duration=hold_seconds, settle_timeout=45,
        axial_pose=True, local_normal=GREEN,
        pose_check=lambda sample: tilt(sample) <= tolerance,
    )
    angle = tilt(pose)
    rig.report(f"Released | all six targets 0.10 kgf | green-axis error {angle:.2f}°")
    rig.log(dict(kind="released", tilt_deg=angle, targets_kgf=BASE.tolist()))
    if interrupted:
        raise KeyboardInterrupt
    if not verified:
        raise AlignmentNotReached(
            f"Baseline restored. Green-axis error after release {angle:.2f}°, "
            f"required ≤{tolerance:g}° throughout verification."
        )
    rig.log(dict(kind="result", reason="Green-axis alignment verified at baseline", tilt_deg=angle))


@click.command()
@click.option("--data", type=click.Path(path_type=Path), default=DATA)
@click.option("--config", type=click.Path(path_type=Path), default=HERE.parents[1] / "pins.json")
@click.option("--max-tension", type=click.FloatRange(0.1, MAX_TARGET_KGF, min_open=True), default=MAX_TARGET_KGF, show_default=True,
              help="Maximum absolute tension per motor (kgf).")
@click.option("--tolerance", type=click.FloatRange(0, 90, min_open=True, max_open=True),
              default=8.0, show_default=True)
@click.option("--max-trials", type=click.IntRange(1), default=36, show_default=True)
@click.option("--hold-seconds", type=click.IntRange(1, 30), default=3, show_default=True)
def main(data, config, max_tension, tolerance, max_trials, hold_seconds):
    """Align the green axes by tuning opposing tendon pairs under load."""
    if not np.isfinite([max_tension, tolerance]).all():
        raise click.UsageError("Tension and tolerance must be finite.")
    data.mkdir(parents=True, exist_ok=True)
    limits = (max_tension - BASE[0]) * np.eye(len(BASE))

    def run(rig):
        align_and_release(rig, max_tension=max_tension, tolerance=tolerance,
              max_trials=max_trials, hold_seconds=hold_seconds)

    run_on_hardware(run, actions=limits, mode="align", data=data, config=config, seed=0,
                    force_cap=FORCE_CUTOFF_KGF)


if __name__ == "__main__":
    main()
