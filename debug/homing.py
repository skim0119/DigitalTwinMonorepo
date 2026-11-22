"""Settle SpiRob at 0.1 kgf per tendon and measure vertical alignment on /poses.

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
MAX_RUN_S = 10.0
SETTLE_S = 0.5
FORCE_TOL_KGF = 0.02
POSE_MAX_AGE_S = 0.25


def positive_float(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return number


def relative_pose(msg):
    points = {body.name: body.pose.position for body in msg.poses}
    if not {"arm_base", "arm_tip"} <= points.keys():
        return None
    base, tip = points["arm_base"], points["arm_tip"]
    xyz = tuple(1000 * (getattr(tip, axis) - getattr(base, axis)) for axis in "xyz")
    return xyz if all(math.isfinite(value) for value in xyz) else None


def home(controller, poll_pose, write_row, xy_tol_mm, deadline):
    start = deadline - MAX_RUN_S
    first = last = None
    stable = deque()
    next_sample = start
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
            ready = controller.ready()
            if not ready or pose is None or now - pose[0] > POSE_MAX_AGE_S:
                if last is not None:
                    return "sensor data lost", first, last
                continue
            controller.command(TARGETS, True, min(0.15, deadline - now))
            forces = tuple(channel[0] for channel in controller.pid_status())
        except RuntimeError as error:
            return f"controller fault: {error}", first, last
        if not all(math.isfinite(force) for force in forces):
            return "invalid load-cell reading", first, last
        stamp, xyz = pose
        x, y, z = xyz
        lateral = math.hypot(x, y)
        error = max(abs(force - target) for force, target in zip(forces, TARGETS))
        write_row((now - start, *xyz, lateral, *forces))
        last = (now, lateral, z, forces)
        if lateral > xy_tol_mm or z >= 0 or error > FORCE_TOL_KGF:
            stable.clear()
            continue
        if not stable or stamp > stable[-1][0]:
            stable.append((stamp, xyz))
        while len(stable) > 1 and stamp - stable[1][0] >= SETTLE_S:
            stable.popleft()
        if stamp - stable[0][0] >= SETTLE_S:
            if all(math.dist(xyz, point) <= xy_tol_mm for _, point in stable):
                return "homed", first, last
    return "timeout", first, last


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "pins.json")
    parser.add_argument("--xy-tol-mm", type=positive_float, default=2.0)
    parser.add_argument("--max-tension-kg", type=positive_float, default=4.5)
    args = parser.parse_args()

    import rclpy
    from control_core import open_controller, safety_limits
    from motion_capture_tracking_interfaces.msg import NamedPoseArray
    from rclpy.qos import qos_profile_sensor_data

    settings = json.loads(args.config.read_text())
    force_limit, rate_limit = safety_limits(settings)
    force_limit = min(force_limit, args.max_tension_kg)
    step_limit = settings.get("max_tightening_steps", 3000)
    if force_limit <= TARGETS[0] or type(step_limit) is not int or step_limit <= 0:
        parser.error("Force limit must exceed 0.1 kgf and tightening limit must be positive.")

    pose = None
    source_stamp = None

    def receive(msg):
        nonlocal pose, source_stamp
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        xyz = relative_pose(msg)
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
            raise SystemExit("The Experiment API or another collector owns the hardware.") from None
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
                writer.writerow(("elapsed_s", "x_mm", "y_mm", "z_mm", "lateral_mm",
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
                        controller, poll_pose, writer.writerow, args.xy_tol_mm, deadline,
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
        stamp, lateral, z, forces = last
        print(f"Last tip: lateral={lateral:.2f} mm, z={z:.2f} mm")
        print("Last tensions (kgf): " + ", ".join(f"{force:.3f}" for force in forces))
        print(f"Last sample at {stamp - (deadline - MAX_RUN_S):.2f} s")
    else:
        print("No paired mocap and load-cell measurements.")
    return 0 if result == "homed" else 1


if __name__ == "__main__":
    sys.exit(main())
