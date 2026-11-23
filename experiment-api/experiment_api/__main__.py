"""Launch a single local service on the Pi or run against simulated hardware."""
import argparse
import fcntl
import json
import os
from functools import partial
from pathlib import Path

import uvicorn

from .api import create_app
from .hardware import Calibration, Hardware
from .runner import Experiments
from .video_recorder import start_recording


def main():
    parser = argparse.ArgumentParser(description="Spirob Experiment API")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--simulate", action="store_true")
    mode.add_argument("--force-only", action="store_true",
                      help="Enable hardware force control using the configured motor step limits")
    mode.add_argument("--calibration", type=Path, help="Enable hardware using a verified calibration JSON")
    parser.add_argument("--data", type=Path, default=Path.home() / "spirob-experiments")
    parser.add_argument("--port", type=int, default=5052)
    parser.add_argument("--camera-index", type=int, default=0, help="Webcam device index")
    parser.add_argument("--no-webcam", action="store_true", help="Skip webcam recording")
    args = parser.parse_args()
    if args.camera_index < 0:
        parser.error("Camera index must be nonnegative.")
    try:
        hardware = None
        if args.calibration or args.force_only:
            from control_core import safety_limits

            pins = Path(os.environ.get("SPIROB_PINS_CONFIG", "pins.json"))
            settings = json.loads(pins.read_text())
            limits = safety_limits(settings)
            calibration = (Calibration.force_only(settings, limits) if args.force_only else
                           Calibration.load(json.loads(args.calibration.read_text()), limits))
            hardware = Hardware(calibration)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error(f"Invalid hardware configuration: {error}")
    webcam = None
    if hardware and not args.no_webcam:
        webcam = partial(start_recording, camera_index=args.camera_index)
    args.data.mkdir(parents=True, exist_ok=True)
    lock_path = (Path.home() / ".spirob-experiment-hardware.lock" if hardware
                 else args.data / ".experiment-api.lock")
    with lock_path.open("a") as lock, (args.data / ".data.lock").open("a") as data_lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.flock(data_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error(f"Experiment API already owns {args.data} or the hardware.")
        uvicorn.run(create_app(Experiments(args.data, hardware, webcam=webcam)), host="127.0.0.1", port=args.port)


if __name__ == "__main__":
    main()
