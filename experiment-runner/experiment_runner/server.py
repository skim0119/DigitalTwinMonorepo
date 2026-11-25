"""Serve uploaded force tables and synchronously return pose-aligned measurements."""
import argparse
import configparser
import csv
import fcntl
import json
import os
import shutil
import threading
from dataclasses import replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from .hardware import Calibration, Hardware
from .runner import Experiments, POSE_FIELDS
from .table import MAX_CSV, TENDONS, parse_table


def export(folder, mapping):
    result = folder / "result.csv"
    with (folder / "poses.csv").open() as poses, (folder / "controls.csv").open() as controls, result.open("w", newline="") as out:
        writer = csv.writer(out)
        fields = ("target_kgf", "load_kgf", "sample_s")
        writer.writerow([*POSE_FIELDS, *(f"{t}_{f}" for t in TENDONS for f in fields)])
        samples = csv.DictReader(controls)
        sample = next(samples, None)
        latest = {}
        for pose in csv.DictReader(poses):
            while sample and float(sample["elapsed_s"]) <= float(pose["elapsed_s"]):
                latest[sample["tendon"]] = (sample["setpoint"], sample["load_kgf"], sample["elapsed_s"])
                sample = next(samples, None)
            writer.writerow([*(pose[k] for k in POSE_FIELDS),
                             *(v for motor in mapping for v in latest.get(TENDONS[motor], ("", "", "")))])
    return result


def handler(runner, mapping):
    busy = threading.Lock()
    order = tuple(mapping.index(i) for i in range(6))

    class Upload(BaseHTTPRequestHandler):
        def do_POST(self):
            if self.path not in ("/run", "/run?unit=N", "/run?unit=kgf"):
                self.send_error(400, "Use POST /run with an optional unit=N or unit=kgf")
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= MAX_CSV:
                    raise ValueError("Expected a CSV of at most 256 KiB")
                self.connection.settimeout(15)
                raw = self.rfile.read(length)
                if len(raw) != length:
                    raise ValueError("Incomplete upload")
                table = parse_table(raw, "force", "kgf")
                table = replace(table, values=tuple(tuple(row[i] for i in order) for row in table.values))
                if not busy.acquire(False):
                    self.send_error(409, "Experiment already running")
                    return
                try:
                    run, = runner.submit(table, raw, "LAN upload", 1, body_prefix="arm_")
                    runner.worker.join()
                    run = runner.get(run["id"])
                    if run["status"] != "complete":
                        self.send_error(503, run["status"], run.get("error"))
                        return
                    result = export(runner.root / run["id"], mapping)
                finally:
                    busy.release()
            except ValueError as error:
                self.send_error(400, str(error))
                return
            self.send_response(200)
            self.send_header("Content-Type", "text/csv")
            self.send_header("Content-Disposition", 'attachment; filename="result.csv"')
            self.send_header("Content-Length", str(result.stat().st_size))
            self.end_headers()
            with result.open("rb") as stream:
                shutil.copyfileobj(stream, self.wfile)
    return Upload


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("config.ini"))
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--runs", type=Path, default=Path("server-runs"))
    args = parser.parse_args()
    config = configparser.ConfigParser()
    with args.config.open() as stream:
        config.read_file(stream)
    mapping = tuple(map(int, config["actuation"]["motors"].split(",")))
    if sorted(mapping) != list(range(6)):
        parser.error("motors must list each motor 0 through 5 exactly once")
    from control_core import safety_limits
    pins = json.loads(Path(os.environ.get("SPIROB_PINS_CONFIG", "pins.json")).read_text())
    hardware = Hardware(Calibration.force_only(pins, safety_limits(pins)))
    with (Path.home() / ".spirob-experiment-hardware.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        runner = Experiments(args.runs, hardware)
        try:
            with ThreadingHTTPServer((args.host, args.port), handler(runner, mapping)) as server:
                runner.start_mocap()
                server.serve_forever()
        finally:
            runner.close()
