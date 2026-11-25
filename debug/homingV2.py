"""Counter-bend SpiRob and verify vertical alignment at 0.1 kgf per tendon.

The mocap frame must have +Z aligned with gravity-up. This standalone debug
run stops force control on exit.
"""

import argparse
import csv
import fcntl
import json
import math
import os
import sys
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TARGETS = (0.1,) * 6
MAX_RUN_S = 100.0
SETTLE_S = 0.5
FORCE_TOL_KGF = 0.02
POSE_MAX_AGE_S = 0.25
PAIRS = ((0, 5), (1, 3), (2, 4))
OPPOSITE = {a: b for pair in PAIRS for a, b in (pair, pair[::-1])}
PAYOUT_KGF = 0.05
DEFAULT_RELEASE_S = 3.0
INITIAL_PROBE_KGF = 1.0
PROBE_KGF = 0.35
CORRECTION_KGF = 0.65
FINAL_S = 2.5
RETRY_SETTLE_S = 1.5
MAX_PULSE_S = 2.0


def residual(feedback):
    return max(math.hypot(*feedback[:2]), math.hypot(*feedback[2:]))


def pull(tendon, force):
    targets = list(TARGETS)
    targets[tendon] = force
    targets[OPPOSITE[tendon]] = PAYOUT_KGF
    return targets


class CounterBend:
    def __init__(self, known_tendon=None, orientation_weight=2.0, release_s=DEFAULT_RELEASE_S):
        self.orientation_weight = orientation_weight
        self.release_s = release_s
        self.directions = [known_tendon] if known_tendon is not None else [a for a, _ in PAIRS]
        self.known = known_tendon is not None
        self.index = 0
        self.reverse = False
        self.sweep_start = None
        self.before = None
        self.interruptions = 0
        self.best_settled = math.inf
        self.pull_start = self.release_start = None
        self.window = deque()
        self.baseline_since = None
        self.force = CORRECTION_KGF if self.known else INITIAL_PROBE_KGF

    def interrupt(self):
        active = self.pull_start is not None or self.release_start is not None
        if active:
            self.interruptions += 1
            self.before = None  # No completed trial to score or reverse.
        self.pull_start = self.release_start = self.baseline_since = None
        self.window.clear()
        return active

    def score(self, pose):
        return math.hypot(*pose[:2], *(v * math.sqrt(self.orientation_weight) for v in pose[2:]))

    def command(self, now, deadline, xyz, forces, stamp):
        error = residual(xyz)
        score = self.score(xyz)
        finishing = deadline - now <= self.release_s + FINAL_S
        if self.pull_start is not None:
            age = now - self.pull_start
            if stamp > self.last_stamp:
                self.smoothed += 0.3 * (score - self.smoothed)
                self.last_stamp = stamp
                if self.smoothed < self.best - 0.05:
                    self.best = self.smoothed
                    self.improved = now
            loaded = forces[self.tendon] >= TARGETS[0] + 0.75 * (self.force - TARGETS[0])
            if loaded and self.loaded_at is None:
                self.loaded_at = now
            plateau = self.loaded_at is not None and now - max(self.loaded_at, self.improved) >= 0.7
            worsening = self.smoothed > self.before + 0.5
            if not (finishing or worsening or plateau or age >= MAX_PULSE_S or error <= 1):
                return "pull", pull(self.tendon, self.force)
            self.pull_start = None
            self.release_start = now
        if self.release_start is not None:
            remaining = 1 - (now - self.release_start) / self.release_s
            if remaining > 0:
                return "slow-release", [b + remaining * (a-b) for a,b in
                                        zip(pull(self.tendon, self.force), TARGETS)]
            self.release_start = None

        at_baseline = all(abs(f-b) <= FORCE_TOL_KGF for f,b in zip(forces,TARGETS))
        # Compare poses only under baseline load and after motion has settled.
        if not at_baseline:
            self.baseline_since = None
            self.window.clear()
            return "recover-tension", TARGETS
        if self.baseline_since is None:
            self.baseline_since = stamp
        if not self.window or stamp > self.window[-1][0]:
            self.window.append((stamp, xyz))
        while len(self.window) > 1 and stamp - self.window[1][0] >= SETTLE_S:
            self.window.popleft()
        settled = (self.window and stamp - self.window[0][0] >= SETTLE_S
                   and all(math.dist(point, xyz) <= 0.2 for _, point in self.window))
        if not settled or stamp - self.baseline_since < RETRY_SETTLE_S:
            return "recover-pose", TARGETS
        if error <= 1.0:
            return "verify", TARGETS

        if finishing:
            return "recover", TARGETS

        if self.interruptions >= 2:
            return "repeated feedback interruption", TARGETS
        self.best_settled = min(self.best_settled, score)
        if self.force == PROBE_KGF and score > self.best_settled + max(0.5, 0.2 * self.best_settled):
            return "settled pose regressed", TARGETS

        if self.before is not None:
            self.interruptions = 0
            gained = self.before - score >= 0.2
            if not gained and not self.reverse and not self.known:
                self.reverse = True
            else:
                if gained:
                    self.directions[self.index] = self.tendon
                self.reverse = False
                self.index = (self.index + 1) % len(self.directions)
                if self.index == 0:
                    if self.sweep_start - score < 0.2:
                        return "no retained improvement", TARGETS
                    self.sweep_start = score
                    if not self.known:
                        self.force = PROBE_KGF
        if self.sweep_start is None:
            self.sweep_start = score
        self.tendon = self.directions[self.index]
        if self.reverse:
            self.tendon = OPPOSITE[self.tendon]
        self.before = self.best = self.smoothed = score
        self.last_stamp = stamp
        self.pull_start = self.improved = now
        self.loaded_at = self.baseline_since = None
        self.window.clear()
        return "pull", pull(self.tendon, self.force)


def positive_float(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return number


def relative_pose(msg, tip_axis=None):
    poses = {body.name: body.pose for body in msg.poses}
    if not {"arm_base", "arm_tip"} <= poses.keys():
        return None
    base, tip = poses["arm_base"].position, poses["arm_tip"].position
    xyz = tuple(1000 * (getattr(tip, axis) - getattr(base, axis)) for axis in "xyz")
    if not all(math.isfinite(value) for value in xyz):
        return None
    if tip_axis is None:
        return xyz
    q = poses["arm_tip"].orientation
    norm = math.hypot(q.x, q.y, q.z, q.w)
    if not math.isfinite(norm) or norm < 1e-6:
        return None
    x, y, z, w = (component / norm for component in (q.x, q.y, q.z, q.w))
    # Each column is one local tip axis expressed in the mocap frame.
    dx, dy, dz = {
        "x": (1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)),
        "y": (2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)),
        "z": (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)),
    }[tip_axis]
    lateral = math.hypot(dx, dy)
    tilt = math.degrees(math.atan2(lateral, abs(dz)))
    # Project onto the +Z hemisphere so either vertical axis gives zero tilt.
    scale = math.copysign(tilt / lateral, dz) if lateral else 0.0
    return (*xyz, dx * scale, dy * scale)


def home(
    controller,
    poll_pose,
    write_row,
    xy_tol_mm,
    deadline,
    known_tendon=None,
    tilt_tol_deg=None,
    orientation_weight=2.0,
    release_s=DEFAULT_RELEASE_S,
):
    start = deadline - MAX_RUN_S
    first = last = None
    stable = deque()
    verification_started = None
    minimum_verification = 4.0 if known_tendon is not None else RETRY_SETTLE_S
    next_sample = start
    # Joint homing needs measured position and tilt responses for each antagonistic pair.
    sequence = CounterBend(known_tendon if tilt_tol_deg is None else None, orientation_weight, release_s)
    while (remaining := deadline - time.monotonic()) > 0:
        pose = poll_pose(min(0.05, remaining))
        now = time.monotonic()
        if now >= deadline:
            break
        if pose is not None and first is None:
            first = pose[1]
        if now < next_sample:
            continue
        next_sample = now + 0.05
        try:
            if pose is None or now - pose[0] > POSE_MAX_AGE_S:
                if last is not None:
                    return "mocap data lost", first, last
                continue
            if not controller.ready():
                stable.clear()
                interrupted = sequence.interrupt()
                verification_started = None
                if last is not None:
                    # Native PID pauses stale channels while it waits for fresh readings.
                    controller.command(TARGETS, True, min(0.15, deadline - now))
                if interrupted:
                    x, y, z = pose[1][:3]
                    tilt = math.hypot(*pose[1][3:]) if tilt_tol_deg is not None else None
                    write_row((now - start, "pose" if tilt is not None else "position",
                               "feedback-interrupted", x, y, z, math.hypot(x, y), tilt,
                               *TARGETS, *(None,) * 6))
                continue
            xyz = pose[1][:3]
            tilt_xy = pose[1][3:]
            feedback = tuple(value / xy_tol_mm for value in xyz[:2])
            if tilt_tol_deg is not None:
                feedback += tuple(value / tilt_tol_deg for value in tilt_xy)
            forces = tuple(channel[0] for channel in controller.pid_status())
            if len(forces) != 6 or not all(math.isfinite(f) for f in forces):
                return "invalid load-cell reading", first, last
            phase, targets = sequence.command(now, deadline, feedback, forces, pose[0])
            controller.command(targets, True, min(0.15, deadline - now))
        except RuntimeError as error:
            return f"controller fault: {error}", first, last
        if len(forces) != 6 or not all(math.isfinite(force) for force in forces):
            return "invalid load-cell reading", first, last
        stamp = pose[0]
        tilt = math.hypot(*tilt_xy) if tilt_xy else None
        x, y, z = xyz
        lateral = math.hypot(x, y)
        error = max(abs(force - target) for force, target in zip(forces, TARGETS))
        write_row(
            (
                now - start,
                "pose" if tilt_tol_deg is not None else "position",
                phase,
                *xyz,
                lateral,
                tilt,
                *targets,
                *forces,
            )
        )
        last = (now, lateral, z, forces, tilt)
        if phase == "verify" and verification_started is None:
            verification_started = now
        if phase in ("no retained improvement", "repeated feedback interruption", "settled pose regressed"):
            return phase, first, last
        if phase != "verify":
            verification_started = None
        if (
            phase not in ("baseline", "verify")
            or lateral > xy_tol_mm
            or z >= 0
            or error > FORCE_TOL_KGF
            or (tilt_tol_deg is not None and tilt > tilt_tol_deg)
        ):
            stable.clear()
            continue
        if not stable or stamp > stable[-1][0]:
            stable.append((stamp, xyz, tilt_xy))
        while len(stable) > 1 and stamp - stable[1][0] >= SETTLE_S:
            stable.popleft()
        if stamp - stable[0][0] >= SETTLE_S:
            settle_time = (
                RETRY_SETTLE_S if tilt_tol_deg is not None else minimum_verification
            )
            if phase == "verify" and now - verification_started < settle_time:
                continue
            if all(
                math.dist(xyz, point) <= 0.2 * xy_tol_mm
                and (tilt_tol_deg is None or math.dist(tilt_xy, angle) <= 0.2 * tilt_tol_deg)
                for _, point, angle in stable
            ):
                return "homed", first, last
    return "timeout", first, last


def save_diagnostics(path, result, elapsed, xy_tol, tilt_tol, orientation_weight=2.0):
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure
    from textwrap import fill

    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    fig = Figure(figsize=(12, 11), layout="constrained")
    FigureCanvasAgg(fig)
    fig.suptitle(f"{path.stem}  |  {elapsed:.2f} s\n{fill(result, 95)}")
    if rows:
        def values(key):
            return [float(row[key]) if row[key] else math.nan for row in rows]

        t = values("elapsed_s")
        labels = ["Lateral (mm)"] + (["Tilt (deg)"] if tilt_tol is not None else [])
        axes = fig.subplots(len(labels) + 5, sharex=True)
        for ax, label, key, limit in zip(axes, labels, ("lateral_mm", "tip_tilt_deg"),
                                         (xy_tol, tilt_tol)):
            samples = values(key)
            ax.plot(t, samples, ".-", markersize=2)
            if key == "lateral_mm":
                best = min((i for i, v in enumerate(samples) if math.isfinite(v)),
                           key=samples.__getitem__, default=None)
                if best is not None:
                    ax.plot(t[best], samples[best], "o", color="tab:red",
                            label=f"Min {samples[best]:.2f} mm at {t[best]:.1f} s")
            ax.axhline(limit, color="gray", linestyle="--", label=f"Tolerance {limit:g}")
            ax.legend(loc="upper right")
            ax.set_ylabel(label)
        score_ax = axes[len(labels)]
        scores = [math.hypot(float(row["lateral_mm"]) / xy_tol,
                             math.sqrt(orientation_weight) * float(row["tip_tilt_deg"]) / tilt_tol
                             if tilt_tol else 0.0) for row in rows]
        score_ax.plot(t, scores, color="0.6")
        score_ax.set_ylabel("Weighted error")
        starts = [i for i, row in enumerate(rows) if row["phase"] == "pull"
                  and (i == 0 or rows[i-1]["phase"] != "pull")]
        for n, start in enumerate(starts):
            end = next((i for i in range(start+1, len(rows)) if rows[i]["phase"] != "pull"), len(rows))
            best = min(range(start, end), key=scores.__getitem__)
            next_start = starts[n+1] if n+1 < len(starts) else len(rows)
            settled = next((i for i in range(end, next_start)
                            if rows[i]["phase"] in ("verify", "no retained improvement")), next_start)
            for i, marker, label in ((start, "o", "Before"), (best, "^", "Best under pull"),
                                     (settled, "x", "Settled")):
                if i < len(rows):
                    score_ax.scatter(t[i], scores[i], marker=marker,
                                     color={"o":"tab:blue","^":"tab:green","x":"tab:red"}[marker],
                                     label=label if n == 0 else None)
        if starts:
            score_ax.legend(loc="upper right", ncols=3, fontsize=8)
        phase_ax = axes[len(labels)+1]
        phases = list(dict.fromkeys(row["phase"] for row in rows))
        indices = [phases.index(row["phase"]) for row in rows]
        phase_ax.step(t, indices, where="post", color="0.4")
        phase_ax.scatter(t, indices, s=3)
        phase_ax.set_yticks(range(len(phases)), phases, fontsize=8)
        for ax, pair in zip(axes[-3:], PAIRS):
            ax.axhspan(TARGETS[0] - FORCE_TOL_KGF, TARGETS[0] + FORCE_TOL_KGF,
                       color="0.9")
            for color, motor in zip(("tab:blue", "tab:orange"), pair):
                ax.plot(t, values(f"tendon_{motor+1}_kgf"), ".-", color=color,
                        markersize=2, label=f"Motor {motor} measured")
                ax.step(t, values(f"target_{motor+1}_kgf"), where="post",
                        color=color, linestyle="--", label=f"Motor {motor} target")
            ax.set_ylabel("Tension (kgf)")
            ax.legend(loc="upper right", ncols=2, fontsize=8)
        axes[-1].set_xlabel("Elapsed time (s)")
        for ax in axes:
            ax.set_xlim(0, max(elapsed, t[-1], 0.1))
            ax.grid(axis="x", alpha=0.2)
    else:
        fig.text(0.5, 0.5, "No complete samples recorded", ha="center")
    output = path.with_suffix(".png")
    fig.savefig(output, dpi=140)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "pins.json")
    parser.add_argument("--xy-tol-mm", type=positive_float, default=2.0)
    parser.add_argument("--max-tension-kg", type=positive_float, default=4.5)
    parser.add_argument(
        "--known-tendon",
        type=int,
        choices=range(1, 7),
        help="Use a previously measured inward tendon with --position-only; joint homing probes all three antagonistic pairs.",
    )
    orientation = parser.add_mutually_exclusive_group()
    orientation.add_argument(
        "--tip-axis",
        choices=("x", "y", "z"),
        help="Calibrated local arm_tip axis for tilt correction (default: z).",
    )
    orientation.add_argument(
        "--position-only",
        dest="tip_axis",
        action="store_const",
        const=None,
        help="Home tip position only.",
    )
    parser.set_defaults(tip_axis="z")
    parser.add_argument("--tilt-tol-deg", type=positive_float, default=5.0)
    parser.add_argument("--orientation-weight", type=positive_float, default=5.0,
                        help="Weight on squared tilt error relative to squared lateral error, each normalized by its tolerance (default: 2).")
    parser.add_argument("--release-s", type=positive_float, default=DEFAULT_RELEASE_S,
                        help="Seconds to ramp both pair targets back to baseline (default: 3).")
    args = parser.parse_args()
    tilt_tol_deg = args.tilt_tol_deg if args.tip_axis else None
    known_tendon = None if args.known_tendon is None else args.known_tendon - 1

    import rclpy
    from control_core import open_controller, safety_limits
    from motion_capture_tracking_interfaces.msg import NamedPoseArray
    from rclpy.qos import qos_profile_sensor_data

    settings = json.loads(args.config.read_text())
    force_limit, rate_limit = safety_limits(settings)
    force_limit = min(force_limit, args.max_tension_kg)
    step_limit = settings.get("max_tightening_steps", 3000)
    required_force = CORRECTION_KGF if known_tendon is not None and tilt_tol_deg is None else INITIAL_PROBE_KGF
    if force_limit <= required_force or type(step_limit) is not int or step_limit <= 0:
        parser.error(
            f"Force limit must exceed {required_force} kgf and tightening limit must be positive."
        )

    pose = None
    source_stamp = None

    def receive(msg):
        nonlocal pose, source_stamp
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        xyz = relative_pose(msg, args.tip_axis)
        if xyz is not None and (source_stamp is None or stamp > source_stamp):
            source_stamp = stamp
            pose = (time.monotonic(), xyz)

    def poll_pose(timeout):
        rclpy.spin_once(node, timeout_sec=timeout)
        return pose

    with (Path.home() / ".spirob-experiment-hardware.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit("Another collector owns the hardware.") from None
        output = ROOT / "output" / "debug"
        output.mkdir(parents=True, exist_ok=True)
        path = output / datetime.now(timezone.utc).strftime(
            "homing_%Y%m%dT%H%M%S.%fZ.csv"
        )
        rclpy.init()
        node = rclpy.create_node("spirob_homing")
        try:
            node.create_subscription(
                NamedPoseArray, "/poses", receive, qos_profile_sensor_data
            )
            with path.open("w", newline="") as stream:
                print(f"Performance CSV: {path}", flush=True)
                writer = csv.writer(stream)
                writer.writerow(
                    (
                        "elapsed_s",
                        "stage",
                        "phase",
                        "x_mm",
                        "y_mm",
                        "z_mm",
                        "lateral_mm",
                        "tip_tilt_deg",
                        *(f"target_{i}_kgf" for i in range(1, 7)),
                        *(f"tendon_{i}_kgf" for i in range(1, 7)),
                    )
                )
                deadline = time.monotonic() + MAX_RUN_S
                controller = open_controller(
                    settings,
                    [(-step_limit, step_limit)] * 6,
                    port=int(os.getenv("SPIROB_MONITOR_PORT", "5050")),
                    max_force=force_limit,
                    max_rate=rate_limit,
                    max_tightening_steps=step_limit,
                )
                try:
                    result, first, last = home(
                        controller,
                        poll_pose,
                        writer.writerow,
                        args.xy_tol_mm,
                        deadline,
                        known_tendon,
                        tilt_tol_deg,
                        args.orientation_weight,
                        args.release_s,
                    )
                finally:
                    controller.stop()
        finally:
            node.destroy_node()
            rclpy.try_shutdown()

    elapsed = time.monotonic() - (deadline - MAX_RUN_S)
    print(f"{result} after {elapsed:.2f} s")
    if first is not None:
        print(
            f"Initial tip: lateral={math.hypot(*first[:2]):.2f} mm, z={first[2]:.2f} mm"
        )
    if last is not None:
        stamp, lateral, z, forces, tilt = last
        print(f"Last tip: lateral={lateral:.2f} mm, z={z:.2f} mm")
        if tilt is not None:
            print(f"Last tip tilt: {tilt:.2f} deg")
        print("Last tensions (kgf): " + ", ".join(f"{force:.3f}" for force in forces))
        print(f"Last sample at {stamp - (deadline - MAX_RUN_S):.2f} s")
    else:
        print("No complete mocap and load-cell measurements.")
        print("Homing requires arm_base and a valid arm_tip pose in one frame.")
    try:
        print(f"Diagnostic plot: {save_diagnostics(path, result, elapsed, args.xy_tol_mm, tilt_tol_deg, args.orientation_weight)}")
    except Exception as error:
        print(f"Could not export diagnostic plot: {error}", file=sys.stderr)
    return 0 if result == "homed" else 1


if __name__ == "__main__":
    sys.exit(main())
