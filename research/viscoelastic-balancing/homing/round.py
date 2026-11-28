"""Visit each opposing motor pair once using measured pose error."""
import math
import sys
from pathlib import Path

import click
import numpy as np

# Support python homing/round.py from the experiment checkout.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from homing import BASE, DATA, HERE, MOTOR_PAIRS, SettleTimeout, run_on_hardware
from probe_model import rotation


LOCAL_Y = (0, 1, 0)


def pose_errors(pose, *, vertical_axis="z"):
    tip_y_in_base = rotation(pose).apply(LOCAL_Y)
    angle = np.degrees(np.arccos(np.clip(abs(tip_y_in_base[1]), 0, 1)))
    horizontal = [index for index, axis in enumerate("xyz") if axis != vertical_axis]
    return np.linalg.norm(pose[horizontal]), angle


def cost(pose, *, position_weight=1.0, tilt_weight=1.0, vertical_axis="z"):
    mm, degrees = pose_errors(pose, vertical_axis=vertical_axis)
    return np.hypot(np.sqrt(position_weight) * mm / 5,
                    np.sqrt(tilt_weight) * degrees / 8)


PULL_KGF = 0.50
DEFAULT_MAX_PINGS = 5
DEFAULT_MIN_IMPROVEMENT = 0.05


def pull_targets(motor):
    targets = BASE.copy()
    targets[motor] += PULL_KGF
    return targets


def run_round(rig, *, settle_timeout,
              position_weight=1.0, tilt_weight=1.0, vertical_axis="z",
              max_pings=DEFAULT_MAX_PINGS, min_improvement=DEFAULT_MIN_IMPROVEMENT):
    def loss_at(pose):
        return cost(pose, position_weight=position_weight, tilt_weight=tilt_weight,
                    vertical_axis=vertical_axis)

    pose, _ = rig.hold(BASE, "baseline", settle_timeout=settle_timeout, axial_pose=True, local_normal=LOCAL_Y)
    initial_pose = pose.copy()

    def report_improvement(final_pose):
        initial_mm, initial_tilt = pose_errors(initial_pose, vertical_axis=vertical_axis)
        final_mm, final_tilt = pose_errors(final_pose, vertical_axis=vertical_axis)
        initial_loss, final_loss = loss_at(initial_pose), loss_at(final_pose)
        rig.report("Round improvement at baseline (positive means closer to home)")
        rig.report(f"  horizontal error {initial_mm:.2f} → {final_mm:.2f} mm "
                   f"({initial_mm - final_mm:+.2f} mm)")
        rig.report(f"  Tilt error {initial_tilt:.2f} → {final_tilt:.2f}° "
                   f"({initial_tilt - final_tilt:+.2f}°)")
        rig.report(f"  Loss {initial_loss:.3f} → {final_loss:.3f} "
                   f"({initial_loss - final_loss:+.3f})")

    rig.report(f"Measuring horizontal separation with mocap {vertical_axis.upper()} vertical")
    for pair in MOTOR_PAIRS:
        baseline_loss = loss_at(pose)
        rig.report(f"Pair {pair[0]}/{pair[1]} | baseline loss {baseline_loss:.3f}")
        for motor in pair:
            targets = pull_targets(motor)
            improved = False
            for ping in range(1, max_pings + 1):
                before_loss = loss_at(pose)
                rig.report(f"Ping {ping}/{max_pings} | "
                           f"m{motor} +{PULL_KGF:.2f} kgf")
                try:
                    rig.hold(targets, "pull", settle_timeout=settle_timeout, axial_pose=True, local_normal=LOCAL_Y)
                except SettleTimeout as error:
                    rig.report(f"m{motor} | {error}. Returning to baseline.")
                    pose, _ = rig.hold(BASE, "return", settle_timeout=settle_timeout, axial_pose=True, local_normal=LOCAL_Y)
                    break

                pose, _ = rig.hold(BASE, "return", settle_timeout=settle_timeout, axial_pose=True, local_normal=LOCAL_Y)
                loss = loss_at(pose)
                mm, degrees = pose_errors(pose, vertical_axis=vertical_axis)
                rig.report(f"Settled baseline loss {before_loss:.3f} → {loss:.3f} "
                           f"| horizontal error {mm:.2f} mm, {degrees:.2f}° tilt")
                if mm <= 5 and (tilt_weight == 0 or degrees <= 8):
                    report_improvement(pose)
                    return
                improvement = before_loss - loss
                if improvement <= 0 or improvement < min_improvement:
                    rig.report(f"m{motor} | improvement {improvement:+.3f} below repeat criterion")
                    break
                improved = True
            if improved:
                break
    report_improvement(pose)


@click.command()
@click.option("--data", type=click.Path(path_type=Path), default=DATA)
@click.option("--config", type=click.Path(path_type=Path), default=HERE.parents[1] / "pins.json")
@click.option("--settle-timeout", type=click.IntRange(min=1), default=90, show_default=True)
@click.option("--position-weight", type=click.FloatRange(min=0), default=1.0,
              show_default=True, help="Weight on squared horizontal error normalized by 5 mm.")
@click.option("--tilt-weight", type=click.FloatRange(min=0), default=1.0,
              show_default=True, help="Weight on squared base-to-tip Y-axis angle normalized by 8 degrees.")
@click.option("--vertical-axis", type=click.Choice(["x", "y", "z"]), default="z",
              show_default=True, help="Vertical axis in the mocap world frame.")
@click.option("--max-pings", type=click.IntRange(min=1), default=DEFAULT_MAX_PINGS,
              show_default=True, help="Maximum pings per motor direction.")
@click.option("--min-improvement", type=click.FloatRange(min=0), default=DEFAULT_MIN_IMPROVEMENT,
              show_default=True, help="Minimum settled loss reduction required to repeat a ping.")
def main(data, config, settle_timeout, position_weight, tilt_weight, vertical_axis, max_pings, min_improvement):
    """Run one measured-loss homing round, pulling one tendon at a time."""
    if not all(map(math.isfinite, (position_weight, tilt_weight))) or max(position_weight, tilt_weight) == 0:
        raise click.UsageError("Use finite weights with at least one positive weight.")
    if not math.isfinite(min_improvement):
        raise click.BadParameter("Must be finite.", param_hint="--min-improvement")
    data.mkdir(parents=True, exist_ok=True)

    def run(rig):
        run_round(rig, settle_timeout=settle_timeout, position_weight=position_weight,
                  tilt_weight=tilt_weight, vertical_axis=vertical_axis,
                  max_pings=max_pings, min_improvement=min_improvement)

    actions = np.array([pull_targets(motor) - BASE for pair in MOTOR_PAIRS for motor in pair])
    run_on_hardware(run, actions=actions, mode="round",
                    data=data, config=config, seed=0, world_translation=True,
                    world_orientation=False)


if __name__ == "__main__":
    main()
