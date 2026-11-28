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


def pose_errors(pose, goal, *, vertical_axis="z"):
    normal = rotation(pose).apply([0, 0, 1])
    reference_normal = (np.eye(3)["xyz".index(vertical_axis)] if goal is None
                        else rotation(goal).apply([0, 0, 1]))
    angle = np.degrees(np.arccos(np.clip(abs(normal @ reference_normal), 0, 1)))
    horizontal = [index for index, axis in enumerate("xyz") if axis != vertical_axis]
    return np.linalg.norm(pose[horizontal]), angle


def cost(pose, goal, *, position_weight=1.0, tilt_weight=0.0, vertical_axis="z"):
    mm, degrees = pose_errors(pose, goal, vertical_axis=vertical_axis)
    return np.hypot(np.sqrt(position_weight) * mm / 5,
                    np.sqrt(tilt_weight) * degrees / 8)


PULL_KGF = 0.50
RELEASE_KGF = 0.10
PINGS_PER_DIRECTION = 3


def paired_targets(motor, partner):
    targets = BASE.copy()
    targets[motor] += PULL_KGF
    targets[partner] -= RELEASE_KGF
    return targets


def run_round(rig, *, goal, settle_timeout,
              position_weight=1.0, tilt_weight=0.0, vertical_axis="z"):
    def loss_at(pose):
        return cost(pose, goal, position_weight=position_weight, tilt_weight=tilt_weight,
                    vertical_axis=vertical_axis)

    pose, _ = rig.hold(BASE, "baseline", settle_timeout=settle_timeout, axial_pose=True)
    initial_pose = pose.copy()

    def report_improvement(final_pose):
        initial_mm, initial_tilt = pose_errors(initial_pose, goal, vertical_axis=vertical_axis)
        final_mm, final_tilt = pose_errors(final_pose, goal, vertical_axis=vertical_axis)
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
        for motor, partner in (pair, pair[::-1]):
            targets = paired_targets(motor, partner)
            direction_start_loss = loss_at(pose)
            for ping in range(1, PINGS_PER_DIRECTION + 1):
                before_loss = loss_at(pose)
                rig.report(f"Ping {ping}/{PINGS_PER_DIRECTION} | "
                           f"m{motor} +{PULL_KGF:.2f}, m{partner} -{RELEASE_KGF:.2f} kgf")
                try:
                    rig.hold(targets, "pull", settle_timeout=settle_timeout, axial_pose=True)
                except SettleTimeout as error:
                    rig.report(f"m{motor}/m{partner} | {error}. Returning to baseline.")
                    pose, _ = rig.hold(BASE, "return", settle_timeout=settle_timeout, axial_pose=True)
                    break

                pose, _ = rig.hold(BASE, "return", settle_timeout=settle_timeout, axial_pose=True)
                loss = loss_at(pose)
                mm, degrees = pose_errors(pose, goal, vertical_axis=vertical_axis)
                rig.report(f"Settled baseline loss {before_loss:.3f} → {loss:.3f} "
                           f"| horizontal error {mm:.2f} mm, {degrees:.2f}° tilt")
                if mm <= 5 and (tilt_weight == 0 or degrees <= 8):
                    report_improvement(pose)
                    return
            else:
                final_loss = loss_at(pose)
                improved = final_loss < direction_start_loss
                verdict = "improved" if improved else "did not improve"
                rig.report(f"m{motor}/m{partner} {verdict} after {PINGS_PER_DIRECTION} pings "
                           f"| loss {direction_start_loss:.3f} → {final_loss:.3f}")
                if improved:
                    break
    report_improvement(pose)


@click.command()
@click.option("--data", type=click.Path(path_type=Path), default=DATA)
@click.option("--reference", type=click.Path(path_type=Path),
              help="Optional saved orientation for nonzero tilt weight.")
@click.option("--config", type=click.Path(path_type=Path), default=HERE.parents[1] / "pins.json")
@click.option("--settle-timeout", type=click.IntRange(min=1), default=90, show_default=True)
@click.option("--position-weight", type=click.FloatRange(min=0), default=1.0,
              show_default=True, help="Weight on squared horizontal error normalized by 5 mm.")
@click.option("--tilt-weight", type=click.FloatRange(min=0), default=0.0,
              show_default=True, help="Weight on squared tilt error normalized by 8 degrees.")
@click.option("--vertical-axis", type=click.Choice(["x", "y", "z"]), default="z",
              show_default=True, help="Vertical axis in the mocap world frame.")
def main(data, reference, config, settle_timeout, position_weight, tilt_weight, vertical_axis):
    """Run one measured-loss homing round, using opposing tendon pairs."""
    if not all(map(math.isfinite, (position_weight, tilt_weight))) or max(position_weight, tilt_weight) == 0:
        raise click.UsageError("Use finite weights with at least one positive weight.")
    goal = load_home_reference(reference) if reference is not None else None
    data.mkdir(parents=True, exist_ok=True)

    def run(rig):
        run_round(rig, goal=goal,
                  settle_timeout=settle_timeout, position_weight=position_weight,
                  tilt_weight=tilt_weight, vertical_axis=vertical_axis)

    actions = np.array([paired_targets(motor, partner) - BASE
                        for pair in MOTOR_PAIRS for motor, partner in (pair, pair[::-1])])
    run_on_hardware(run, actions=actions, mode="round",
                    data=data, config=config, seed=0, world_translation=True,
                    world_orientation=reference is None)


if __name__ == "__main__":
    main()
