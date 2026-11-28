"""Visit each opposing motor pair once using measured pose error."""
import sys
from pathlib import Path

import click
import numpy as np

# Support python homing/round.py from the experiment checkout.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from homing import BASE, DATA, HERE, MOTOR_PAIRS, SettleTimeout, load_home_reference, run_on_hardware
from probe_model import cost, pose_errors


def run_round(rig, *, goal, step, max_pull, settle_timeout):
    pose, _ = rig.hold(BASE, "baseline", settle_timeout=settle_timeout)
    for pair in MOTOR_PAIRS:
        baseline_loss = cost(pose, goal)
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
                    loaded_pose, _ = rig.hold(targets, "pull", settle_timeout=settle_timeout)
                    loss = cost(loaded_pose, goal)
                    rig.report(f"m{motor} +{offset:.3f} kgf | loss {loss:.3f}")
                    if best_loss - loss < 0.01:
                        if loss > best_loss and best_targets[motor] > BASE[motor]:
                            rig.report(f"m{motor} | backing off to {best_targets[motor]:.3f} kgf")
                            rig.hold(best_targets, "backoff", settle_timeout=settle_timeout)
                        break
                    best_loss = loss
                    best_targets = targets.copy()
                    mm, degrees = pose_errors(loaded_pose, goal)
                    if mm <= 5 and degrees <= 8:
                        break
            except SettleTimeout as error:
                rig.report(f"m{motor} | {error}")
            pose, _ = rig.hold(BASE, "return", settle_timeout=settle_timeout)
            mm, degrees = pose_errors(pose, goal)
            rig.report(f"At baseline | {mm:.2f} mm, {degrees:.2f}°, loss {cost(pose, goal):.3f}")
            if mm <= 5 and degrees <= 8:
                return
            if best_loss < baseline_loss:
                break
            baseline_loss = cost(pose, goal)


@click.command()
@click.option("--data", type=click.Path(path_type=Path), default=DATA)
@click.option("--reference", type=click.Path(path_type=Path))
@click.option("--config", type=click.Path(path_type=Path), default=HERE.parents[1] / "pins.json")
@click.option("--step", type=click.FloatRange(min=0.02, max=0.5), default=0.1, show_default=True)
@click.option("--max-pull", type=click.FloatRange(min=0.02, max=0.5), default=0.5, show_default=True)
@click.option("--settle-timeout", type=click.IntRange(min=1), default=90, show_default=True)
def main(data, reference, config, step, max_pull, settle_timeout):
    """Run one measured-loss homing round, one tendon at a time."""
    goal = load_home_reference(reference or data / "home.json")
    data.mkdir(parents=True, exist_ok=True)

    def run(rig):
        run_round(rig, goal=goal, step=step, max_pull=max_pull,
                  settle_timeout=settle_timeout)

    run_on_hardware(run, actions=max_pull * np.eye(len(BASE)), mode="round",
                    data=data, config=config, seed=0)


if __name__ == "__main__":
    main()
