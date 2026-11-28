"""Collect and use baseline-to-baseline SE(3) probe responses."""

import fcntl
import json
import os
import time
from contextlib import ExitStack
from datetime import datetime, timezone
from functools import partial
from pathlib import Path

import click
import numpy as np
from scipy.spatial.transform import Rotation

from probe_model import ProbeModel, pose_errors, relative, train

HERE = Path(__file__).resolve().parent
DATA = HERE / "data"

BASE = np.full(6, 0.1)
MOTOR_PAIRS = ((0, 5), (1, 3), (2, 4))
PULL_OFFSETS_KGF = (0.10, 0.25, 0.50)
MAX_TENSION_KGF = 3.0
RELEASE_OFFSETS_KGF = (-0.05, -0.08)
INCREMENT_KGF_S = 0.5
DECREMENT_KGF_S = 0.1
MEMORY_SECONDS = (1.0, 8.0)
HOLD_S = 0.6
COMMAND_TTL_S = 0.20
SETTLE_MAX_SAMPLE_GAP_S = 0.15
POSE_MAX_AGE_S = 0.25


def tip_in_base(message):
    bodies = {b.name: b.pose for b in message.poses}
    if not {"arm_base", "arm_tip"} <= bodies.keys():
        return None
    poses = []
    for name in ("arm_base", "arm_tip"):
        p, q = bodies[name].position, bodies[name].orientation
        pose = np.array([1000 * p.x, 1000 * p.y, 1000 * p.z, q.x, q.y, q.z, q.w])
        if not np.isfinite(pose).all() or np.linalg.norm(pose[3:]) < 1e-6:
            return None
        poses.append(pose)
    delta = relative(*poses)
    return np.r_[delta[:3], Rotation.from_rotvec(delta[3:]).as_quat()]


def sweep_actions():
    actions = [np.zeros(6)]
    for a, b in MOTOR_PAIRS:
        for motor, partner in ((a, b), (b, a)):
            for pull in PULL_OFFSETS_KGF:
                action = np.zeros(6)
                action[motor] = pull
                actions.append(action)
                for release in RELEASE_OFFSETS_KGF:
                    paired = action.copy()
                    paired[partner] = release
                    actions.append(paired)
    return np.array(actions)


class Rig:
    def __init__(self, controller, spin, log, force_limit):
        self.controller, self.spin, self.log = controller, spin, log
        self.limit = force_limit
        self.stamp = self.pose = None
        self.memory = np.zeros((2, 6))
        self.last_pose = self.last_command = self.last = None
        self.targets = BASE.copy()
        self.cycle = 0

    def receive(self, msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        pose = tip_in_base(msg)
        if pose is not None and (self.stamp is None or stamp > self.stamp):
            self.stamp, self.pose = stamp, pose

    def refresh_command(self):
        sent_at = time.monotonic()
        if self.last_command is not None and sent_at - self.last_command >= COMMAND_TTL_S:
            raise RuntimeError(
                f"Command refresh gap {sent_at - self.last_command:.3f} s "
                "before sending target, after reading controller feedback"
            )
        self.controller.command(self.targets.tolist(), True, COMMAND_TTL_S)
        self.last_command = sent_at

    def hold(self, end, phase, duration=HOLD_S, goal=None, *, deadline=None):
        if not np.isfinite(end).all() or min(end) < 0 or max(end) >= self.limit:
            raise ValueError("Probe target outside configured force limits")
        print(
            f"  {phase}: waiting for feedback and {duration:g} s at target", flush=True
        )
        begin = time.monotonic()
        next_report = begin + 3
        load_gap = None
        stable = None
        poses = []
        within_goal = False
        phase_end = begin + 30 if deadline is None else min(begin + 30, deadline)
        while (remaining := phase_end - time.monotonic()) > 0:
            previous = self.stamp
            poll_start = time.monotonic()
            self.spin(min(0.05, remaining))
            now = time.monotonic()
            if now >= phase_end:
                break
            fresh_pose = self.stamp != previous
            if fresh_pose:
                self.last_pose = now
            if (
                self.last_command is not None
                and now - self.last_command >= COMMAND_TTL_S
            ):
                raise RuntimeError(
                    f"{phase}: command refresh gap {now - self.last_command:.3f} s "
                    f"(before ROS poll: {poll_start - self.last_command:.3f} s, "
                    f"ROS poll and callback: {now - poll_start:.3f} s)"
                )
            ready = self.controller.ready()
            if self.last is not None and now - self.last_pose >= POSE_MAX_AGE_S:
                raise RuntimeError(
                    f"{phase}: no new valid arm_base/arm_tip pose for "
                    f"{now - self.last_pose:.3f} s (load cells ready={ready})"
                )
            report = now >= next_report
            if report:
                next_report = now + 3
                if not ready or not fresh_pose:
                    print(
                        f"  {phase} {now - begin:.0f}s: new pose={fresh_pose}, "
                        f"load cells ready={ready}",
                        flush=True,
                    )
            if not ready and self.last is not None:
                if load_gap is None:
                    load_gap = now
                    stable = None
                    poses.clear()
                    self.log(
                        dict(kind="load_gap", phase=phase, cycle=self.cycle, t=now)
                    )
                if now - load_gap >= 1.0:
                    raise RuntimeError(
                        f"{phase}: load-cell feedback unavailable for 1 s"
                    )
            if not ready or not fresh_pose:
                if self.last is not None:
                    # Native PID pauses stale channels and faults on persistent loss.
                    self.refresh_command()
                continue
            if load_gap is not None:
                self.log(
                    dict(
                        kind="load_recovered",
                        phase=phase,
                        cycle=self.cycle,
                        t=now,
                        gap_s=now - load_gap,
                    )
                )
                load_gap = None
            if self.last is not None and now - self.last >= SETTLE_MAX_SAMPLE_GAP_S:
                stable = None
                poses.clear()
            forces = np.array([item[0] for item in self.controller.pid_status()])
            if forces.shape != (6,) or not np.isfinite(forces).all():
                raise RuntimeError("Invalid force feedback")
            dt = min(0.05, now - self.last) if self.last is not None else 0
            elapsed = now - self.last if self.last is not None else 0
            self.last = now
            self.memory += (-np.expm1(-elapsed / np.array(MEMORY_SECONDS)))[:, None] * (
                forces - BASE - self.memory
            )
            self.targets += np.clip(
                end - self.targets, -DECREMENT_KGF_S * dt, INCREMENT_KGF_S * dt
            )
            self.refresh_command()
            self.log(
                dict(
                    kind="sample",
                    cycle=self.cycle,
                    phase=phase,
                    t=now,
                    stamp_ns=self.stamp,
                    pose=self.pose.tolist(),
                    target=self.targets.tolist(),
                    force=forces.tolist(),
                )
            )
            settled = (
                np.allclose(self.targets, end, atol=1e-9, rtol=0)
                and max(abs(forces - end)) <= 0.02
            )
            if report:
                worst = int(np.argmax(abs(forces - end)))
                print(
                    f"  {phase} {now - begin:.0f}s: largest target error on m{worst}, "
                    f"measured {forces[worst]:.3f} / goal {end[worst]:.3f} kgf",
                    flush=True,
                )
            if not settled:
                stable = None
                poses.clear()
                continue
            if stable is None:
                stable, within_goal = now, True
            poses.append(self.pose)
            if goal is not None:
                mm, deg = pose_errors(self.pose, goal)
                within_goal &= mm <= 5 and deg <= 8
            if now - stable >= duration:
                samples = np.array(poses)
                mean = np.r_[
                    samples[:, :3].mean(axis=0),
                    Rotation.from_quat(samples[:, 3:]).mean().as_quat(),
                ]
                return mean, within_goal
        if deadline is not None and time.monotonic() >= deadline:
            raise TimeoutError(f"Time budget expired during {phase}")
        raise RuntimeError(f"Timed out waiting for {phase} feedback")

    def probe(self, start, action, block, *, deadline=None):
        memory = self.memory.ravel().copy()
        probe_deadline = (
            None if deadline is None else deadline - ramp_seconds(-action) - 2 * HOLD_S
        )
        try:
            self.hold(BASE + action, "probe", deadline=probe_deadline)
        except TimeoutError:
            self.hold(BASE, "return", deadline=deadline)
            raise RuntimeError("Time budget reached during probe. Returned to baseline.") from None
        end, _ = self.hold(BASE, "return", deadline=deadline)
        self.log(
            dict(
                kind="cycle",
                cycle=self.cycle,
                block=block,
                start=start.tolist(),
                action=action.tolist(),
                memory=memory.tolist(),
                end=end.tolist(),
                change=relative(start, end).tolist(),
            )
        )
        self.cycle += 1


def capture_reference(rig, *, reference):
    pose, _ = rig.hold(BASE, "reference", duration=1.0)
    with reference.open("x") as stream:
        json.dump(dict(frame="arm_base/arm_tip", pose=pose.tolist()), stream)
    print("Saved home pose to", reference)


def run_sweep(rig, *, actions, blocks, seed):
    rng = np.random.default_rng(seed)
    for block in range(blocks):
        started = time.monotonic()
        for n, i in enumerate(rng.permutation(len(actions)), 1):
            offsets = " ".join(f"m{j}{v:+.2f}" for j, v in enumerate(actions[i]) if v)
            print(
                f"Block {block + 1}/{blocks}, probe {n}/{len(actions)}: "
                f"{offsets + ' kgf' if offsets else 'baseline only'}",
                flush=True,
            )
            start, _ = rig.hold(BASE, "baseline")
            rig.probe(start, actions[i], block)
            elapsed = time.monotonic() - started
            if n < len(actions):
                remaining = elapsed / n * (len(actions) - n)
                print(
                    f"  Recorded. Block elapsed {elapsed:.0f}s, "
                    f"about {remaining:.0f}s remaining",
                    flush=True,
                )
        print(
            f"Completed sweep block {block + 1}/{blocks} in {elapsed:.1f}s",
            flush=True,
        )


def ramp_seconds(offset):
    return np.maximum(offset / INCREMENT_KGF_S, -offset / DECREMENT_KGF_S).max()


def estimated_probe_seconds(action):
    return ramp_seconds(action) + ramp_seconds(-action) + 3 * HOLD_S


def run_prediction(rig, *, model, goal, max_probes=None, max_seconds=60):
    deadline = time.monotonic() + max_seconds
    step = 0

    def duration(action):
        if np.max(action) > max(PULL_OFFSETS_KGF):
            return np.inf
        estimate = estimated_probe_seconds(action)
        return estimate if estimate + HOLD_S < deadline - time.monotonic() else np.inf

    try:
        while time.monotonic() < deadline:
            pose, reached = rig.hold(BASE, "verify", goal=goal, deadline=deadline)
            mm, deg = pose_errors(pose, goal)
            print(f"After {step} probes: {mm:.2f} mm, {deg:.2f} deg", flush=True)
            if reached:
                return
            if step == max_probes:
                raise RuntimeError("Probe budget exhausted before homing")
            action = model.choose(pose, rig.memory.ravel(), goal, duration=duration)
            if action is not None and np.isfinite(duration(action)):
                rig.probe(pose, action, 0, deadline=deadline)
                step += 1
    except TimeoutError as error:
        raise RuntimeError(str(error)) from None
    raise RuntimeError("Time budget expired before homing")


def export_homing_diagnostics(path, goal):
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    with path.open() as stream:
        records = [json.loads(line) for line in stream]
    samples = [row for row in records if row["kind"] == "sample"]
    cycles = [row for row in records if row["kind"] == "cycle"]
    outcome = next((row["reason"] for row in reversed(records)
                    if row["kind"] in ("result", "abort")), "Run interrupted")

    figure = Figure(figsize=(11, 9), layout="constrained")
    FigureCanvasAgg(figure)
    figure.suptitle(f"Predictive homing · {outcome}", wrap=True)
    if samples:
        elapsed = np.array([row["t"] for row in samples])
        elapsed -= elapsed[0]
        errors = np.array([pose_errors(np.asarray(row["pose"]), goal) for row in samples])
        position, orientation, tension, probes = figure.subplots(4, 1)
        for index, (ax, label, limit) in enumerate(
            ((position, "Position error (mm)", 5), (orientation, "Rotation error (°)", 8))
        ):
            ax.plot(elapsed, errors[:, index])
            ax.axhline(limit, color="0.35", linestyle="--", label="Home tolerance")
            ax.set_ylabel(label)
            ax.legend(loc="upper right")

        measured = np.array([row["force"] for row in samples])
        target = np.array([row["target"] for row in samples])
        for motor in range(measured.shape[1]):
            line, = tension.plot(elapsed, measured[:, motor], label=f"m{motor}")
            tension.plot(elapsed, target[:, motor], color=line.get_color(), linestyle="--")
        tension.set_ylabel("Tension (kgf)")
        tension.set_xlabel("Elapsed time (s)")
        tension.set_title("Measured (solid) and commanded (dashed)")
        tension.legend(ncol=6)

        if cycles:
            for index, (label, scale) in enumerate((("Position / 5 mm", 5), ("Rotation / 8°", 8))):
                before = [pose_errors(np.asarray(row["start"]), goal)[index] / scale for row in cycles]
                after = [pose_errors(np.asarray(row["end"]), goal)[index] / scale for row in cycles]
                for number, (start, end) in enumerate(zip(before, after), 1):
                    offset = -0.08 if index == 0 else 0.08
                    probes.plot([number + offset] * 2, [start, end], color=f"C{index}", alpha=0.7)
                    probes.plot(number + offset, start, "o", color=f"C{index}", mfc="none")
                    probes.plot(number + offset, end, "o", color=f"C{index}",
                                label=label if number == 1 else None)
            probes.axhline(1, color="0.35", linestyle="--")
            probes.set_xlabel("Completed probe")
            probes.set_ylabel("Error / tolerance")
            probes.set_title("Before (open) and after (filled) return to baseline")
            probes.legend()
        else:
            probes.set_axis_off()
            probes.text(0.5, 0.5, "No completed probes", ha="center", transform=probes.transAxes)
    else:
        figure.text(0.5, 0.5, "No usable feedback samples", ha="center")
    output = path.with_suffix(".png")
    figure.savefig(output, dpi=150)
    print(f"Saved homing diagnostics to {output}", flush=True)


def save_homing_diagnostics(path, goal):
    try:
        export_homing_diagnostics(path, goal)
    except Exception as error:
        click.echo(f"Could not save homing diagnostics: {error}", err=True)


@click.command(context_settings={"help_option_names": ["-h", "--help"]})
@click.argument("mode", type=click.Choice(["reference", "sweep", "train", "predict"]))
@click.option("--data", type=click.Path(path_type=Path), default=DATA)
@click.option("--extra-data", type=click.Path(path_type=Path, exists=True, file_okay=False),
              multiple=True, help="Additional sweep directory for training. Repeat to combine directories.")
@click.option("--model", type=click.Path(path_type=Path))
@click.option("--reference", type=click.Path(path_type=Path))
@click.option(
    "--config", type=click.Path(path_type=Path), default=HERE.parents[1] / "pins.json"
)
@click.option("--blocks", type=click.IntRange(min=1), default=3, show_default=True)
@click.option("--seed", type=int, default=0)
@click.option("--max-probes", type=click.IntRange(min=1), help="Optional probe count limit.")
@click.option("--max-seconds", type=click.IntRange(min=1), default=60, show_default=True,
              help="Time budget for predictive homing.")
def main(*, mode, data, extra_data, model, reference, config, blocks, seed, max_probes, max_seconds):
    """Collect and use baseline-to-baseline SE(3) probe responses."""
    if extra_data and mode != "train":
        raise click.UsageError("--extra-data requires train")
    data.mkdir(parents=True, exist_ok=True)

    model = model or data / "probe_model.npz"
    reference = reference or data / "home.json"
    goal = None
    if mode == "train":
        train(data, model, extra_data=extra_data)
        return
    elif mode == "predict":
        model = ProbeModel.load(model)
        try:
            reference = json.loads(reference.read_text())
        except FileNotFoundError:
            raise click.ClickException(
                f"Home reference missing: {reference}. Place the tip at the desired home "
                "pose and run homing.py reference, or use --reference with an existing file."
            ) from None
        goal = np.asarray(reference["pose"], dtype=float)
        if (
            reference["frame"] != "arm_base/arm_tip"
            or goal.shape != (7,)
            or not np.isfinite(goal).all()
            or np.linalg.norm(goal[3:]) < 1e-6
        ):
            raise click.UsageError("Invalid home reference")
        actions = model.actions[np.max(model.actions, axis=1) <= max(PULL_OFFSETS_KGF)]
        if not len(actions):
            raise click.UsageError("Model has no probes within the current pull limit")
        run = partial(run_prediction, model=model, goal=goal, max_probes=max_probes,
                      max_seconds=max_seconds)
    elif mode == "sweep":
        actions = sweep_actions()
        run = partial(run_sweep, actions=actions, blocks=blocks, seed=seed)
    else:
        if reference.exists():
            raise click.UsageError(
                "Reference already exists. Select another path with --reference."
            )
        actions = np.zeros((1, 6))
        run = partial(capture_reference, reference=reference)

    run_on_hardware(
        run,
        actions=actions,
        mode=mode,
        data=data,
        config=config,
        seed=seed,
        goal=goal,
    )


def run_on_hardware(run, *, actions, mode, data, config, seed, goal=None):
    import rclpy
    from control_core import open_controller, safety_limits
    try:
        from motion_capture_tracking_interfaces.msg import NamedPoseArray
    except ModuleNotFoundError as error:
        if error.name not in {"motion_capture_tracking_interfaces",
                              "motion_capture_tracking_interfaces.msg"}:
            raise
        raise click.ClickException(
            "Missing ROS message package motion_capture_tracking_interfaces. "
            "Source the SpiRob ROS workspace with "
            "source ~/ros2_spirob_ws/install/setup.bash, then retry."
        ) from None
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    settings = json.loads(config.read_text())
    limit, rate = safety_limits(settings)
    limit = min(limit, MAX_TENSION_KGF)
    steps = settings.get("max_tightening_steps", 3000)
    peaks = BASE + actions
    if (
        not np.isfinite(peaks).all()
        or peaks.min() < 0
        or max(BASE.max(), peaks.max()) >= limit
        or type(steps) is not int
        or steps <= 0
    ):
        raise click.UsageError("Probe actions exceed configured hardware bounds")
    with ExitStack() as diagnostics, ExitStack() as stack:
        lock = stack.enter_context(
            (Path.home() / ".spirob-experiment-hardware.lock").open("a")
        )
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise click.ClickException(
                "SpiRob hardware is busy. Identify the owner with "
                'lsof "$HOME/.spirob-experiment-hardware.lock". '
                "Retry after that process exits."
            ) from None
        rclpy.init()
        stack.callback(rclpy.try_shutdown)
        node = rclpy.create_node("spirob_probe")
        stack.callback(node.destroy_node)
        date = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        path = data / f"{mode}-{date}.jsonl"
        stream = stack.enter_context(path.open("x", buffering=1))
        if goal is not None:
            diagnostics.callback(save_homing_diagnostics, path, goal)

        def log(record):
            stream.write(json.dumps(record, allow_nan=False) + "\n")

        log(
            dict(
                kind="session",
                version=1,
                frame="arm_base/arm_tip",
                pose_units="mm, quaternion xyzw",
                baseline_kgf=BASE.tolist(),
                increment_kgf_s=INCREMENT_KGF_S,
                decrement_kgf_s=DECREMENT_KGF_S,
                memory_seconds=MEMORY_SECONDS,
                hold_s=HOLD_S,
                seed=seed,
                mode=mode,
            )
        )
        controller = open_controller(
            settings,
            [(-steps, steps)] * 6,
            port=int(os.getenv("SPIROB_MONITOR_PORT", "5050")),
            max_force=limit,
            max_rate=rate,
            max_tightening_steps=steps,
        )
        stack.callback(controller.stop)
        rig = Rig(
            controller, lambda dt: rclpy.spin_once(node, timeout_sec=dt), log, limit
        )
        node.create_subscription(
            NamedPoseArray,
            "/poses",
            rig.receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        print("Recording", path, flush=True)
        try:
            run(rig)
            if goal is not None:
                log(dict(kind="result", reason="Home verified", t=time.monotonic()))
        except RuntimeError as error:
            log(
                dict(
                    kind="abort", cycle=rig.cycle, reason=str(error), t=time.monotonic()
                )
            )
            raise click.ClickException(
                f"Stopped in cycle {rig.cycle}: {error}. Recording: {path}"
            ) from None


if __name__ == "__main__":
    main()
