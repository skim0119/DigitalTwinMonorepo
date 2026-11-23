"""Adapt the operator-installed debug homing routine to the API controller."""

import runpy
import time
from types import SimpleNamespace


def load_homing(script):
    routine = runpy.run_path(str(script))
    home = routine["home"]
    relative_pose = routine["relative_pose"]
    timeout = routine["MAX_RUN_S"]

    def run(experiments):
        hardware = experiments.hardware
        if routine["CORRECTION_KGF"] >= hardware.calibration.max_force_kgf:
            raise ValueError("Homing force must be below the configured tension limit.")

        def poll_pose(delay):
            if experiments.cancel.wait(delay):
                raise InterruptedError("Homing cancelled.")
            with experiments.lock:
                samples = experiments.home_poses.copy()
            if len(samples) != 2:
                return None
            received, stamp, frame, _ = samples["arm_tip"]
            if samples["arm_base"][1:3] != (stamp, frame):
                return None
            bodies = []
            for name, (_, _, _, values) in samples.items():
                x, y, z = values[:3]
                bodies.append(SimpleNamespace(name=name, pose=SimpleNamespace(
                    position=SimpleNamespace(x=x, y=y, z=z),
                )))
            position = relative_pose(SimpleNamespace(poses=bodies))
            return (received, position) if position is not None else None

        start = time.monotonic()
        try:
            hardware.open_at_position(experiments.cancel)
            result, _, last = home(hardware.controller, poll_pose, lambda _: None, 1.0, start + timeout)
        finally:
            hardware.finish()

        homed = result == "homed" and not experiments.cancel.is_set()
        report = {"homed": homed, "result": result, "elapsed_s": time.monotonic() - start}
        if last is not None:
            _, lateral, z, forces, _ = last
            report.update(lateral_mm=lateral, z_mm=z, tensions_kgf=list(forces))
        return report

    return run
