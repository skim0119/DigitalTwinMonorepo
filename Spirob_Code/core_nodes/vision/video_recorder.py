"""Optional background webcam recording for experiments."""

import logging
import math
from datetime import datetime
from pathlib import Path
from threading import Event, Thread

logger = logging.getLogger(__name__)


def start_recording(directory, *, camera_index=0, filename=None):
    """Start recording in directory and return a callable that stops and saves it."""
    stop = Event()
    worker = Thread(target=_record, args=(Path(directory), camera_index, stop, filename), daemon=True)
    worker.start()

    def stop_recording():
        stop.set()
        worker.join()

    return stop_recording


def _record(directory, camera_index, stop, filename):
    try:
        import cv2
    except ImportError:
        logger.info("Webcam recording requires OpenCV")
        return

    camera = None
    writer = None
    temporary = None
    frames = 0
    try:
        if stop.is_set():
            return
        camera = cv2.VideoCapture(camera_index)
        if not camera.isOpened() or stop.is_set():
            return
        ok, frame = camera.read()
        if not ok or stop.is_set():
            return

        fps = camera.get(cv2.CAP_PROP_FPS)
        if not math.isfinite(fps) or fps <= 0:
            fps = 30.0
        height, width = frame.shape[:2]
        directory.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S.%f%z")
        output = directory / (filename or f"webcam-{timestamp}.mp4")
        temporary = directory / f".{output.stem}.partial.mp4"
        writer = cv2.VideoWriter(str(temporary), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
        if not writer.isOpened():
            logger.warning("Could not open webcam recording at %s", output)
            return

        while ok and not stop.is_set():
            writer.write(frame)
            frames += 1
            ok, frame = camera.read()
    except (OSError, cv2.error):
        logger.exception("Webcam recording stopped")
    finally:
        if writer is not None:
            writer.release()
        if camera is not None:
            camera.release()
        if temporary is not None:
            try:
                if frames and temporary.is_file() and temporary.stat().st_size > 0:
                    temporary.replace(output)
                else:
                    temporary.unlink(missing_ok=True)
            except OSError:
                logger.exception("Could not finalize webcam recording")
