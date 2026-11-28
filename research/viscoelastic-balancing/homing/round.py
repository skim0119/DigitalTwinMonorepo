"""Visit each opposing motor pair once using measured pose error."""
import math
import sys
from pathlib import Path

import click
import numpy as np

# Support python homing/round.py from the experiment checkout.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from homing import BASE, DATA, HERE, MOTOR_PAIRS, SettleTimeout, load_home_reference, run_on_hardware
from probe_model import rotation


def pose_errors(pose, goal):
    normal = rotation(pose).apply([0, 0, 1])
    reference_normal = rotation(goal).apply([0, 0, 1])
    angle = np.degrees(np.arccos(np.clip(abs(normal @ reference_normal), 0, 1)))
    return np.linalg.norm(pose[:3] - goal[:3]), angle


def cost(pose, goal, *, position_weight=1.0, tilt_weight=1.0):
    mm, degrees = pose_errors(pose, goal)
    return np.hypot(np.sqrt(position_weight) * mm / 5,
                    np.sqrt(tilt_weight) * degrees / 8)


def run_round(rig, *, goal, step, max_pull, settle_timeout,
              position_weight=1.0, tilt_weight=1.0):
    def loss_at(pose):
        return cost(pose, goal, position_weight=position_weight, tilt_weight=tilt_weight)

    pose, _ = rig.hold(BASE, "baseline", settle_timeout=settle_timeout, axial_pose=True)
    initial_pose = pose.copy()

    def report_improvement(final_pose):
        initial_mm, initial_tilt = pose_errors(initial_pose, goal)
        final_mm, final_tilt = pose_errors(final_pose, goal)
        initial_loss, final_loss = loss_at(initial_pose), loss_at(final_pose)
        rig.report("Round improvement at baseline (positive means closer to home)")
        rig.report(f"  Position error {initial_mm:.2f} → {final_mm:.2f} mm "
                   f"({initial_mm - final_mm:+.2f} mm)")
        rig.report(f"  Tilt error {initial_tilt:.2f} → {final_tilt:.2f}° "
                   f"({initial_tilt - final_tilt:+.2f}°)")
        rig.report(f"  Loss {initial_loss:.3f} → {final_loss:.3f} "
                   f"({initial_loss - final_loss:+.3f})")

    for pair in MOTOR_PAIRS:
        baseline_loss = loss_at(pose)
        rig.report(f"Pair {pair[0]}/{pair[1]} | baseline loss {baseline_loss:.3f}")
        for motor in pair:
            targets = BASE.copy()
            best_targets = BASE.copy()
            best_loss = baseline_loss
            offset = 0.0
            try:
                while offset < max_pull:
                    offset = min(offset + step, max_pull)
                    targets[motor] = BASE[motor] + offset
                    loaded_pose, _ = rig.hold(targets, "pull", settle_timeout=settle_timeout, axial_pose=True)
                    loss = loss_at(loaded_pose)
                    rig.report(f"m{motor} +{offset:.3f} kgf | loss {loss:.3f}")
                    if loss >= best_loss:
                        rig.report(f"m{motor} | loss stopped decreasing ({best_loss:.3f} → {loss:.3f})")
                        if loss > best_loss and best_targets[motor] > BASE[motor]:
                            rig.report(f"m{motor} | backing off to {best_targets[motor]:.3f} kgf")
                            rig.hold(best_targets, "backoff", settle_timeout=settle_timeout, axial_pose=True)
                        break
                    best_loss = loss
                    best_targets = targets.copy()
                else:
                    rig.report(f"m{motor} | reached +{max_pull:.3f} kgf pull limit")
            except SettleTimeout as error:
                rig.report(f"m{motor} | {error}")
            pose, _ = rig.hold(BASE, "return", settle_timeout=settle_timeout, axial_pose=True)
            mm, degrees = pose_errors(pose, goal)
            rig.report(f"At baseline | {mm:.2f} mm, {degrees:.2f}° tilt, loss {loss_at(pose):.3f}")
            if mm <= 5 and degrees <= 8:
                report_improvement(pose)
                return
            if best_loss < baseline_loss:
                break
            baseline_loss = loss_at(pose)
    report_improvement(pose)


@click.command()
@click.option("--data", type=click.Path(path_type=Path), default=DATA)
@click.option("--reference", type=click.Path(path_type=Path))
@click.option("--config", type=click.Path(path_type=Path), default=HERE.parents[1] / "pins.json")
@click.option("--step", type=click.FloatRange(min=0.02, max=0.5), default=0.1, show_default=True)
@click.option("--max-pull", type=click.FloatRange(min=0, min_open=True), default=0.5,
              show_default=True, help="Maximum pull offset above baseline in kgf.")
@click.option("--settle-timeout", type=click.IntRange(min=1), default=90, show_default=True)
@click.option("--position-weight", type=click.FloatRange(min=0), default=1.0,
              show_default=True, help="Weight on squared position error normalized by 5 mm.")
@click.option("--tilt-weight", type=click.FloatRange(min=0), default=1.0,
              show_default=True, help="Weight on squared tilt error normalized by 8 degrees.")
def main(data, reference, config, step, max_pull, settle_timeout, position_weight, tilt_weight):
    """Run one measured-loss homing round, one tendon at a time."""
    if not math.isfinite(max_pull):
        raise click.BadParameter("Must be finite.", param_hint="--max-pull")
    if not all(map(math.isfinite, (position_weight, tilt_weight))) or max(position_weight, tilt_weight) == 0:
        raise click.UsageError("Use finite weights with at least one positive weight.")
    goal = load_home_reference(reference or data / "home.json")
    data.mkdir(parents=True, exist_ok=True)

    def run(rig):
        run_round(rig, goal=goal, step=step, max_pull=max_pull,
                  settle_timeout=settle_timeout, position_weight=position_weight,
                  tilt_weight=tilt_weight)

    run_on_hardware(run, actions=max_pull * np.eye(len(BASE)), mode="round",
                    data=data, config=config, seed=0)


if __name__ == "__main__":
    main()
