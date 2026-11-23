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
BASELINE_S = 0.8
PROBE_S = 0.75
PROBE_ON_S = 0.60
PROBE_KGF = 0.35
CORRECTION_KGF = 0.65
FINAL_S = 2.5
RETRY_SETTLE_S = 1.5
MAX_PULSE_S = 2.0


def pull(tendon, force):
    targets = list(TARGETS)
    targets[tendon] = force
    return targets


class CounterBend:
    def __init__(self, known_tendon=None, *, angle=False):
        self.known_tendon = known_tendon
        self.angle = angle
        self.pulse_start = None
        self.pulse_xy = None
        self.known_direction = None
        self.start = None
        self.probe_starts = {}
        self.responses = {}
        self.returning = False
        self.selected = None

    def command(self, now, deadline, xyz, xy_tol_mm):
        if self.start is None:
            self.start = now
        elapsed = now - self.start
        if elapsed < BASELINE_S:
            return "baseline", TARGETS
        final_s = 4.0 if self.known_tendon is not None else FINAL_S
        if self.returning or deadline - now <= final_s + SETTLE_S:
            return "verify", TARGETS

        probe_time = elapsed - BASELINE_S
        tendon = int(probe_time / PROBE_S)
        if self.known_tendon is None and tendon < len(TARGETS):
            if probe_time - tendon * PROBE_S < PROBE_ON_S:
                self.probe_starts.setdefault(tendon, xyz)
                return "probe", pull(tendon, PROBE_KGF)
            if tendon in self.probe_starts and tendon not in self.responses:
                self.responses[tendon] = tuple(a - b for a, b in zip(xyz, self.probe_starts[tendon]))
            return "release", TARGETS

        x, y, _ = xyz
        lateral = math.hypot(x, y)
        if self.pulse_start is None and lateral > xy_tol_mm:
            if self.known_tendon is not None:
                if self.known_direction is None:
                    self.known_direction = (x, y)
                dx, dy = self.known_direction
                if x * dx + y * dy > 0:
                    self.selected = self.known_tendon
            elif self.responses:
                inward_mm, _, selected = max(
                    (-(x * dx + y * dy) / lateral, -dz, index)
                    for index, (dx, dy, dz) in self.responses.items()
                )
                if inward_mm > 0.5:
                    self.selected = selected
            if self.selected is not None:
                self.pulse_start = now
                self.pulse_xy = (x, y)

        if self.pulse_start is not None:
            x0, y0 = self.pulse_xy
            initial_offset = math.hypot(x0, y0)
            if (now - self.pulse_start < (0.5 if self.angle else MAX_PULSE_S)
                    and -2.0 * initial_offset < x * x0 + y * y0
                    and lateral <= initial_offset + 2.0):
                return "counter-bend", pull(self.selected, PROBE_KGF if self.angle else CORRECTION_KGF)
        self.returning = True
        return "verify", TARGETS

    def retry(self):
        if self.pulse_start is None:
            return False
        self.pulse_start = None
        self.selected = None
        self.returning = False
        return True


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


def home(controller, poll_pose, write_row, xy_tol_mm, deadline, known_tendon=None,
         tilt_tol_deg=None):
    start = deadline - MAX_RUN_S
    first = last = None
    stable = deque()
    verification_started = None
    minimum_verification = 4.0 if known_tendon is not None else RETRY_SETTLE_S
    next_sample = start
    angle_stage = False
    sequence = CounterBend(known_tendon)
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
                if last is not None:
                    # Native PID pauses stale channels while it waits for fresh readings.
                    controller.command(TARGETS, True, min(0.15, deadline - now))
                continue
            xyz = pose[1][:3]
            tilt_xy = pose[1][3:]
            feedback = (*tilt_xy, -1.0) if angle_stage else xyz
            tolerance = tilt_tol_deg if angle_stage else xy_tol_mm
            phase, targets = sequence.command(now, deadline, feedback, tolerance)
            controller.command(targets, True, min(0.15, deadline - now))
            forces = tuple(channel[0] for channel in controller.pid_status())
        except RuntimeError as error:
            return f"controller fault: {error}", first, last
        if not all(math.isfinite(force) for force in forces):
            return "invalid load-cell reading", first, last
        stamp = pose[0]
        tilt = math.hypot(*tilt_xy) if tilt_xy else None
        x, y, z = xyz
        lateral = math.hypot(x, y)
        error = max(abs(force - target) for force, target in zip(forces, TARGETS))
        write_row((now - start, "angle" if angle_stage else "position", phase,
                   *xyz, lateral, tilt, *targets, *forces))
        last = (now, lateral, z, forces, tilt)
        if phase == "verify" and verification_started is None:
            verification_started = now
        if (angle_stage and phase == "verify" and now - verification_started >= RETRY_SETTLE_S
                and error <= FORCE_TOL_KGF and lateral > xy_tol_mm):
            angle_stage = False
            sequence = CounterBend(known_tendon)
            verification_started = None
            stable.clear()
            continue
        residual = tilt if angle_stage else lateral
        if (phase == "verify" and now - verification_started >= RETRY_SETTLE_S
                and residual > tolerance and error <= FORCE_TOL_KGF and z < 0
                and deadline - now > max(FINAL_S, minimum_verification) + SETTLE_S
                and sequence.retry()):
            verification_started = None
            stable.clear()
            continue
        if (phase not in ("baseline", "verify") or lateral > xy_tol_mm or z >= 0
                or error > FORCE_TOL_KGF or (angle_stage and tilt > tilt_tol_deg)):
            stable.clear()
            continue
        if not stable or stamp > stable[-1][0]:
            stable.append((stamp, xyz))
        while len(stable) > 1 and stamp - stable[1][0] >= SETTLE_S:
            stable.popleft()
        if stamp - stable[0][0] >= SETTLE_S:
            settle_time = RETRY_SETTLE_S if tilt_tol_deg is not None else minimum_verification
            if phase == "verify" and now - verification_started < settle_time:
                continue
            if all(math.dist(xyz, point) <= xy_tol_mm for _, point in stable):
                if tilt_tol_deg is None or angle_stage:
                    return "homed", first, last
                angle_stage = True
                sequence = CounterBend(angle=True)
                verification_started = None
                stable.clear()
    return "timeout", first, last


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "pins.json")
    parser.add_argument("--xy-tol-mm", type=positive_float, default=1.0)
    parser.add_argument("--max-tension-kg", type=positive_float, default=4.5)
    parser.add_argument("--known-tendon", type=int, choices=range(1, 7),
                        help="Use a previously measured inward tendon for pulses of up to 2 s.")
    orientation = parser.add_mutually_exclusive_group()
    orientation.add_argument("--tip-axis", choices=("x", "y", "z"),
                             help="Calibrated local arm_tip axis for tilt correction (default: z).")
    orientation.add_argument("--position-only", dest="tip_axis", action="store_const", const=None,
                             help="Home tip position only.")
    parser.set_defaults(tip_axis="z")
    parser.add_argument("--tilt-tol-deg", type=positive_float, default=2.0)
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
    if force_limit <= CORRECTION_KGF or type(step_limit) is not int or step_limit <= 0:
        parser.error("Force limit must exceed 0.65 kgf and tightening limit must be positive.")

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
        path = output / datetime.now(timezone.utc).strftime("homing_%Y%m%dT%H%M%S.%fZ.csv")
        rclpy.init()
        node = rclpy.create_node("spirob_homing")
        try:
            node.create_subscription(NamedPoseArray, "/poses", receive, qos_profile_sensor_data)
            with path.open("w", newline="") as stream:
                print(f"Performance CSV: {path}", flush=True)
                writer = csv.writer(stream)
                writer.writerow(("elapsed_s", "stage", "phase", "x_mm", "y_mm", "z_mm",
                                 "lateral_mm", "tip_tilt_deg",
                                 *(f"target_{i}_kgf" for i in range(1, 7)),
                                 *(f"tendon_{i}_kgf" for i in range(1, 7))))
                deadline = time.monotonic() + MAX_RUN_S
                controller = open_controller(
                    settings, [(-step_limit, step_limit)] * 6,
                    port=int(os.getenv("SPIROB_MONITOR_PORT", "5050")),
                    max_force=force_limit, max_rate=rate_limit,
                    max_tightening_steps=step_limit,
                )
                try:
                    result, first, last = home(
                        controller, poll_pose, writer.writerow, args.xy_tol_mm, deadline, known_tendon, tilt_tol_deg,
                    )
                finally:
                    controller.stop()
        finally:
            node.destroy_node()
            rclpy.try_shutdown()

    elapsed = time.monotonic() - (deadline - MAX_RUN_S)
    print(f"{result} after {elapsed:.2f} s")
    if first is not None:
        print(f"Initial tip: lateral={math.hypot(*first[:2]):.2f} mm, z={first[2]:.2f} mm")
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
    return 0 if result == "homed" else 1


if __name__ == "__main__":
    sys.exit(main())
