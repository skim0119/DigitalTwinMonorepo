"""Contract tests for upload validation, durable trial playback, and hardware units."""
import csv
import io
import tempfile
import threading
import time
import unittest
from pathlib import Path

from fastapi.testclient import TestClient

from .api import create_app
from .hardware import Calibration, Hardware
from .runner import Experiments, POSE_FIELDS
from .table import parse_table

HEADER = b"time_s,tendon_1,tendon_2,tendon_3,tendon_4,tendon_5,tendon_6\n"
ZERO = b"0,0,0,0,0,0,0\n"
EXAMPLE = HEADER + ZERO + b"0.15,2,0,0,0,0,0\n0.30,0,0,0,0,0,0\n"


class TableTest(unittest.TestCase):
    def test_modes_and_interpolation(self):
        self.assertEqual(parse_table(EXAMPLE, "force", "N").at(0.10)[0], 0)
        self.assertEqual(parse_table(EXAMPLE, "displacement", "mm").at(0.075)[0], 1)
        self.assertEqual(parse_table(EXAMPLE, "force", "N").at(0.15)[0], 2)
        for row in (b"0,2,0,0,0,0,0", b"1,nan,0,0,0,0,0", b"1,-1,0,0,0,0,0", b"1,10"):
            with self.subTest(row=row), self.assertRaises(ValueError):
                parse_table(HEADER + ZERO + row, "force", "N")

    def test_calibration_defaults_and_global_caps(self):
        calibration = dict(steps_per_mm=[2] * 6, travel_mm=[[0, 20]] * 6)
        limits = (4.5, 3000)
        config = Calibration.load(calibration, limits)
        self.assertEqual((config.max_force_kgf, config.max_step_rate), limits)
        for overrides in ({"max_force_kgf": 4.6}, {"max_step_rate": 3001}):
            with self.subTest(overrides=overrides), self.assertRaisesRegex(ValueError, "global"):
                Calibration.load({**calibration, **overrides}, limits)

    def test_force_only_uses_configured_limits(self):
        limits = (4.5, 3000)
        config = Calibration.force_only({"max_tightening_steps": 120}, limits)
        with self.assertRaisesRegex(ValueError, "max_tightening_steps"):
            Calibration.force_only({}, limits)
        with self.assertRaisesRegex(ValueError, "calibration file"):
            config.validate(parse_table(EXAMPLE, "displacement", "mm"))

        def open_controller(path, bounds, **options):
            self.assertEqual(bounds, [(-120, 120)] * 6)
            self.assertEqual(options["max_tightening_steps"], 120)
            self.assertEqual((options["max_force"], options["max_rate"]), limits)
            return object()

        table = parse_table(EXAMPLE, "force", "kgf")
        hardware = Hardware(config, open_controller=open_controller)
        hardware.validate(table)
        cancelled = threading.Event()
        cancelled.set()
        hardware.open(table, cancelled)

    def test_hardware_unit_and_speed_contract(self):
        config = Calibration.load(dict(steps_per_mm=[-2, 2, 2, 2, 2, 2],
                                      travel_mm=[[0, 20]] * 6, max_force_kgf=1, max_step_rate=100), (4.5, 3000))
        config.validate(parse_table(EXAMPLE, "displacement", "mm"))
        with self.assertRaisesRegex(ValueError, "force command"):
            config.validate(parse_table(EXAMPLE, "force", "kgf"))
        with self.assertRaisesRegex(ValueError, "speed"):
            config.validate(parse_table(EXAMPLE.replace(b"0.15", b"0.001"), "displacement", "mm"))
        commands = []

        class Controller:
            def ready(self):
                return True

            def command(self, targets, force, ttl):
                commands.append((targets, force, ttl))

            def status(self):
                return [(0, 0, 0)] * 6

            def stop(self):
                commands.append("stop")

        def open_controller(path, bounds, **options):
            self.assertEqual(bounds[0], (-40, 0))
            return Controller()

        hardware = Hardware(config, open_controller=open_controller)
        for mode, unit, expected in (("force", "N", 2 / 9.80665), ("displacement", "mm", -4)):
            table = parse_table(EXAMPLE, mode, unit)
            hardware.open(table, threading.Event())
            now = time.monotonic()
            hardware.update(table, table.at(.15), now, .02, now + .05)
            self.assertAlmostEqual(commands[-1][0][0], expected)
            self.assertEqual(commands[-1][1], mode == "force")
            self.assertGreater(commands[-1][2], 0)
            self.assertLessEqual(commands[-1][2], .05)
            hardware.finish()
            self.assertEqual(commands[-1], "stop")


class ApiTest(unittest.TestCase):
    def wait_for(self, predicate):
        deadline = time.monotonic() + 4
        while not predicate():
            self.assertLess(time.monotonic(), deadline, "Experiment failed to reach expected state")
            time.sleep(.02)

    def test_repeats_stream_download_review_and_restart(self):
        with tempfile.TemporaryDirectory() as folder:
            experiment = Experiments(folder, repeat_pause=.01)
            with TestClient(create_app(experiment)) as client:
                mode = "?mode=displacement&unit=mm"
                self.assertEqual(client.post("/validate" + mode, content=EXAMPLE).json()["interpolation"], "linear")
                self.assertEqual(client.post("/validate" + mode, content=b"x" * 300000).status_code, 413)
                self.assertEqual(client.post("/validate" + mode, content=HEADER).status_code, 422)
                self.assertEqual(client.post("/runs" + mode, content=EXAMPLE, headers={"Origin": "http://example.com"}).status_code, 403)
                self.assertEqual(client.post("/runs" + mode + "&name=../unsafe", content=EXAMPLE).status_code, 422)
                with client.websocket_connect("/events") as socket:
                    self.assertEqual(socket.receive_json()["states"], {})
                    response = client.post("/runs" + mode + "&repeats=2&name=baseline", content=EXAMPLE)
                    self.assertEqual(response.status_code, 202)
                    runs = response.json()
                    self.assertEqual(client.post("/runs" + mode, content=EXAMPLE).status_code, 409)
                    first = runs[0]["id"]
                    frames = []
                    for _ in range(8):
                        message = socket.receive_json()
                        frames.extend(message["frames"])
                        if any(frame[2] == "poses" for frame in frames):
                            break
                    self.assertTrue(any(frame[1] == first and frame[2] == "poses" for frame in frames))
                self.wait_for(lambda: all(row["status"] == "complete" for row in client.get("/runs").json()))
                for run in runs:
                    self.assertTrue(run["id"].endswith("_baseline"))
                    plot = Path(folder) / run["id"] / "diagnostics.png"
                    self.assertEqual(plot.read_bytes()[:8], b"\x89PNG\r\n\x1a\n")
                poses = client.get(f"/runs/{first}/files/poses.csv?download=true")
                self.assertIn("attachment", poses.headers["content-disposition"])
                self.assertFalse(client.get("/runs").json()[0]["has_video"])
                self.assertEqual(client.get(f"/runs/{first}/files/webcam.mp4").status_code, 404)
                parsed = list(csv.DictReader(io.StringIO(poses.text)))
                self.assertEqual(tuple(parsed[0]), POSE_FIELDS)
                self.assertEqual(parsed[0]["body"], "Rigid Body 001")
                self.assertGreater(len(parsed), 3)
                self.assertEqual(client.get(f"/runs/{first}/files/actuation.csv").content, EXAMPLE)
                review = {"include": False, "intervals": [{"body": "Rigid Body 001", "start": 0.05,
                            "end": 0.1, "action": "exclude"}]}
                self.assertEqual(client.put(f"/runs/{first}/review", json=review).json(), review)
                self.assertEqual(client.get(f"/runs/{first}/files/poses.csv").content, poses.content)
            self.assertEqual(Experiments(folder).get(first)["status"], "complete")
            self.assertTrue((Path(folder) / first / "review.json").exists())

    def test_required_body_missing_prevents_hardware_open(self):
        calibration = Calibration.load(dict(steps_per_mm=[2] * 6, travel_mm=[[0, 20]] * 6,
                                            max_force_kgf=1, max_step_rate=100), (4.5, 3000))
        opened = []
        hardware = Hardware(calibration, open_controller=lambda *args, **kwargs: opened.append(True))
        with tempfile.TemporaryDirectory() as folder:
            experiment = Experiments(folder, hardware)
            experiment.on_pose(time.monotonic(), time.time_ns(), "world", "arm_base",
                               (0, 0, 0, 0, 0, 0, 1))
            table = parse_table(EXAMPLE, "force", "N")
            run, = experiment.submit(table, EXAMPLE, "Bending", 1, ("arm_base", "arm_tip"))
            experiment.worker.join(timeout=2)
            result = experiment.get(run["id"])
            self.assertEqual(result["status"], "failed")
            self.assertIn("arm_tip", result["error"])
            self.assertEqual(opened, [])

    def test_prefix_requires_a_fresh_matching_body_and_records_all_matches(self):
        from .runner import Simulated

        pose = (0, 0, 0, 0, 0, 0, 1)

        class HardwareStub(Simulated):
            def validate(self, table):
                pass

            def update(self, table, values, now, dt, end):
                for name in ("arm_mid", "arm_joint_2", "other_body"):
                    experiment.on_pose(now, time.time_ns(), "world", name, pose)
                return super().update(table, values, now, dt, end)

        with tempfile.TemporaryDirectory() as folder:
            experiment = Experiments(folder, HardwareStub())
            table = parse_table(EXAMPLE, "force", "N")
            experiment.on_pose(time.monotonic(), time.time_ns(), "world", "other_body", pose)
            run, = experiment.submit(table, EXAMPLE, "Bending", 1, body_prefix="arm_")
            experiment.worker.join(timeout=5)
            self.assertIn("prefix 'arm_'", experiment.get(run["id"])["error"])

            experiment.on_pose(time.monotonic(), time.time_ns(), "world", "arm_mid", pose)
            run, = experiment.submit(table, EXAMPLE, "Bending", 1, body_prefix="arm_")
            experiment.worker.join(timeout=5)
            self.assertEqual(experiment.get(run["id"])["status"], "complete")
            with (Path(folder) / run["id"] / "poses.csv").open() as stream:
                self.assertEqual({row["body"] for row in csv.DictReader(stream)}, {"arm_mid", "arm_joint_2"})

    def test_hold_end_sampling_uses_only_completed_holds(self):
        from .diagnostics import hold_end_samples

        series = {"arm_tip": [
            {"elapsed_s": str(time), "x_m": str(time)} for time in (.25, .9, 1, 1.5, 3.1)
        ]}
        result = hold_end_samples(series, (0, 1, 2, 3, 4), 3.1)
        self.assertEqual(result["arm_tip"], [
            {"elapsed_s": 1, "x_m": "0.9"}, {"elapsed_s": 2, "x_m": "1.5"}
        ])

    def test_optional_webcam_download_on_completion_and_stop(self):
        def webcam(directory, *, filename):
            return lambda: (directory / filename).write_bytes(b"mock MP4")

        with tempfile.TemporaryDirectory() as folder:
            experiment = Experiments(folder, webcam=webcam)
            with TestClient(create_app(experiment)) as client:
                for mode, unit, cancel in (("force", "N", False), ("displacement", "mm", True)):
                    csv_body = EXAMPLE.replace(b"0.30", b"2.00") if cancel else EXAMPLE
                    response = client.post(f"/runs?mode={mode}&unit={unit}", content=csv_body)
                    self.assertEqual(response.status_code, 202)
                    run_id = response.json()[0]["id"]
                    video_path = f"/runs/{run_id}/files/webcam.mp4?download=true"
                    if cancel:
                        self.wait_for(lambda: experiment.get(run_id)["status"] == "running")
                        self.assertEqual(client.get(video_path).status_code, 404)
                        client.post(f"/runs/{run_id}/stop").raise_for_status()
                    expected = "cancelled" if cancel else "complete"
                    self.wait_for(lambda: experiment.get(run_id)["status"] == expected)
                    self.wait_for(lambda: not experiment.worker.is_alive())
                    self.assertTrue(client.get("/runs").json()[0]["has_video"])
                    if mode == "force" and not cancel:
                        for filename in ("diagnostics_step_end.png", "diagnostics_hysteresis.png"):
                            plot = Path(folder) / run_id / filename
                            self.assertEqual(plot.read_bytes()[:8], b"\x89PNG\r\n\x1a\n")
                    video = client.get(video_path)
                    self.assertEqual(video.content, b"mock MP4")
                    self.assertEqual(video.headers["content-type"], "video/mp4")
                    self.assertIn("attachment", video.headers["content-disposition"])

    def test_stop_retains_partial_record_and_cancels_queued_trials(self):
        with tempfile.TemporaryDirectory() as folder:
            experiment = Experiments(folder)
            with TestClient(create_app(experiment)) as client:
                response = client.post("/runs?mode=force&unit=N&repeats=2", content=EXAMPLE.replace(b"0.30", b"2.00"))
                first, second = [row["id"] for row in response.json()]
                self.wait_for(lambda: experiment.get(first)["status"] == "running")
                self.assertEqual(client.post(f"/runs/{first}/stop").status_code, 200)
                self.wait_for(lambda: experiment.get(second)["status"] == "cancelled")
                self.assertEqual(experiment.get(first)["status"], "cancelled")
                self.assertEqual(client.get(f"/runs/{first}/files/controls.csv").status_code, 200)


if __name__ == "__main__":
    unittest.main()
