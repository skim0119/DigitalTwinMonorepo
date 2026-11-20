"""Persist each trial on the Pi and decouple recording from live preview."""
import csv
import json
import math
import threading
import time
from collections import deque
from datetime import datetime, timezone
from hashlib import sha256
from pathlib import Path
from queue import Empty, Full, Queue
from uuid import uuid4

from .table import TENDONS

POSE_FIELDS = ("elapsed_s", "ros_stamp_ns", "frame", "body", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw")
CONTROL_FIELDS = ("elapsed_s", "tendon", "setpoint", "unit", "load_kgf", "position_steps", "output_steps_s")
ACTIVE = {"queued", "running", "stopping"}


class BusyError(Exception):
    pass


def write_json(path, obj):
    pending = path.with_suffix(".tmp")
    pending.write_text(json.dumps(obj, allow_nan=False) + "\n")
    pending.replace(path)


class Simulated:
    interval = 0.025

    def open(self, _table, _cancel):
        self.position = 0

    def update(self, _table, values, _now, dt, _end):
        target = sum(max(-100, min(100, value)) for value in values)
        self.position += (target - self.position) * min(1, dt * 8)
        return [(value * 0.1, "", "") for value in values]

    def finish(self):
        pass

    def close(self):
        pass


class Experiments:
    def __init__(self, root, hardware=None, *, repeat_pause=0.5):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.hardware = hardware or Simulated()
        self.simulated = hardware is None
        self.repeat_pause = repeat_pause
        self.lock = threading.RLock()
        self.runs = {}
        self.cancel = threading.Event()
        self.mocap_stop = threading.Event()
        self.mocap_ready = threading.Event()
        self.poses = None
        self.last_pose = 0
        self.mocap_error = None
        self.worker = None
        self.mocap_thread = None
        self.preview = deque(maxlen=1200)
        self.sequence = 0
        for path in self.root.glob("*/run.json"):
            run = json.loads(path.read_text())
            if run["status"] in ACTIVE:
                run.update(status="interrupted", error="Service restarted before the trial finished.")
                write_json(path, run)
            self.runs[run["id"]] = run

    def start_mocap(self):
        if self.simulated:
            return
        from .mocap import receive_poses

        def listen():
            try:
                receive_poses(self.on_pose, self.mocap_stop, self.mocap_ready)
            except Exception as error:
                self.mocap_error = str(error)
                self.mocap_ready.set()

        self.mocap_thread = threading.Thread(target=listen, name="mocap", daemon=True)
        self.mocap_thread.start()

    def on_pose(self, now, stamp, frame, name, values):
        with self.lock:
            self.last_pose = now
            if self.poses is not None:
                try:
                    self.poses.put_nowait((now, str(stamp), frame, name, *values))
                except Full:
                    self.pose_overrun = True

    def list(self):
        with self.lock:
            return [run.copy() for run in reversed(self.runs.values())]

    def get(self, run_id):
        with self.lock:
            return self.runs[run_id].copy()

    def mark(self, run, status, error=None):
        with self.lock:
            updated = dict(run, status=status, error=error)
            write_json(self.root / run["id"] / "run.json", updated)
            run.update(updated)

    def submit(self, table, raw, label, repeats):
        with self.lock:
            if self.worker and self.worker.is_alive():
                raise BusyError("A batch is already active.")
            if not self.simulated:
                self.hardware.calibration.validate(table)
            self.cancel.clear()
            batch = uuid4().hex
            sequence = sha256(json.dumps(table.response(), sort_keys=True).encode()).hexdigest()
            runs = []
            for trial in range(1, repeats + 1):
                run_id = uuid4().hex
                folder = self.root / run_id
                folder.mkdir()
                (folder / "actuation.csv").write_bytes(raw)
                run = dict(id=run_id, batch=batch, sequence=sequence, trial=trial, label=label, table=table.response(),
                           simulated=self.simulated, created=datetime.now(timezone.utc).isoformat())
                self.mark(run, "queued")
                self.runs[run_id] = run
                runs.append(run)
            self.worker = threading.Thread(target=self.play, args=(table, runs), name="trial-runner")
            self.worker.start()
            return [run.copy() for run in runs]

    def stop(self, run_id):
        with self.lock:
            run = self.runs[run_id]
            if run["status"] not in ACTIVE:
                raise BusyError("This trial has already finished.")
            self.cancel.set()
            if run["status"] == "running":
                self.mark(run, "stopping")
            return run.copy()

    def emit_preview(self, run_id, kind, data):
        with self.lock:
            self.sequence += 1
            self.preview.append((self.sequence, run_id, kind, data))

    def play(self, table, runs):
        for run in runs:
            if self.cancel.is_set():
                self.mark(run, "cancelled")
                continue
            try:
                self.record(table, run)
            except Exception as error:
                self.cancel.set()
                self.mark(run, "failed", str(error))
            else:
                self.mark(run, "cancelled" if self.cancel.is_set() else "complete")
                if run is not runs[-1]:
                    self.cancel.wait(self.repeat_pause)

    def record(self, table, run):
        if not self.simulated and (self.mocap_error or time.monotonic() - self.last_pose > 1):
            raise RuntimeError(self.mocap_error or "No recent OptiTrack poses. Check the ROS bridge.")
        folder = self.root / run["id"]
        with (folder / "poses.csv").open("w", newline="", buffering=1) as pose_file, \
                (folder / "controls.csv").open("w", newline="", buffering=1) as control_file:
            pose_csv = csv.writer(pose_file)
            control_csv = csv.writer(control_file)
            pose_csv.writerow(POSE_FIELDS)
            control_csv.writerow(CONTROL_FIELDS)
            queue = Queue(maxsize=8192)
            latest = {}
            try:
                self.hardware.open(table, self.cancel)
                if self.cancel.is_set():
                    return
                start = time.monotonic()
                end = start + table.duration
                with self.lock:
                    self.poses = queue
                    self.pose_overrun = False
                    if not self.cancel.is_set():
                        self.mark(run, "running")
                tick = previous = start
                while not self.cancel.is_set():
                    now = time.monotonic()
                    if now >= end:
                        break
                    if self.pose_overrun or (not self.simulated and
                            (self.mocap_error or now - self.last_pose > 1)):
                        raise RuntimeError("Pose recording was interrupted or fell behind.")
                    values = table.at(now - start)
                    outputs = self.hardware.update(table, values, now, max(now - previous, 0.001), end)
                    if outputs is None:
                        break
                    rows = [(now - start, tendon, value, table.unit, *output)
                            for tendon, value, output in zip(TENDONS, values, outputs)]
                    control_csv.writerows(rows)
                    self.emit_preview(run["id"], "controls", rows)
                    if self.simulated:
                        x = self.hardware.position * 0.008
                        self.on_pose(now, time.time_ns(), "world", "Rigid Body 001",
                                     (x, x * math.sin(now - start), 0.14 + abs(x) * 0.5,
                                      0, 0, math.sin(x), math.cos(x)))
                    self.drain(queue, start, end, pose_csv, latest)
                    if latest:
                        self.emit_preview(run["id"], "poses", list(latest.values()))
                        latest.clear()
                    previous = now
                    tick = max(tick + self.hardware.interval, now + self.hardware.interval * 0.25)
                    self.cancel.wait(max(0, min(end, tick) - time.monotonic()))
            finally:
                with self.lock:
                    self.poses = None
                try:
                    self.hardware.finish()
                finally:
                    if "start" in locals():
                        self.drain(queue, start, end, pose_csv, latest)
            if self.pose_overrun:
                raise RuntimeError("Pose recorder could not keep up.")

    @staticmethod
    def drain(queue, start, end, writer, latest):
        while True:
            try:
                timestamp, *pose = queue.get_nowait()
            except Empty:
                return
            if start <= timestamp <= end:
                row = (timestamp - start, *pose)
                writer.writerow(row)
                latest[pose[2]] = row

    def close(self):
        self.cancel.set()
        if self.worker:
            self.worker.join()
        self.mocap_stop.set()
        if self.mocap_thread:
            self.mocap_thread.join()
        self.hardware.close()
