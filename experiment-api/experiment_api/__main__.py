"""Launch a single local service on the Pi or run against simulated hardware."""
import argparse
import fcntl
import json
import logging
import os
import runpy
from functools import partial
from pathlib import Path

import uvicorn

from .api import create_app
from .hardware import Calibration, Hardware
from .runner import Experiments


def main():
    parser = argparse.ArgumentParser(description="Spirob Experiment API")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--simulate", action="store_true")
    mode.add_argument("--calibration", type=Path, help="Enable hardware using a verified calibration JSON")
    parser.add_argument("--data", type=Path, default=Path.home() / "spirob-experiments")
    parser.add_argument("--port", type=int, default=5052)
    parser.add_argument("--camera-index", type=int, default=0, help="Webcam device index")
    parser.add_argument("--no-webcam", action="store_true", help="Skip webcam recording")
    args = parser.parse_args()
    if args.camera_index < 0:
        parser.error("Camera index must be nonnegative.")
    try:
        hardware = Hardware(Calibration.load(json.loads(args.calibration.read_text()))) if args.calibration else None
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error(f"Invalid calibration: {error}")
    webcam = None
    if hardware and not args.no_webcam:
        root = Path(os.environ.get("SPIROB_REPO_ROOT", Path.cwd()))
        source = root / "Spirob_Code/core_nodes/vision/video_recorder.py"
        if source.is_file():
            webcam = partial(runpy.run_path(str(source))["start_recording"], camera_index=args.camera_index)
        else:
            logging.getLogger(__name__).warning("Webcam recorder missing at %s. Set SPIROB_REPO_ROOT.", source)
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
