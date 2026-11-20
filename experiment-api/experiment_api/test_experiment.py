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

    def test_hardware_unit_and_speed_contract(self):
        config = Calibration.load(dict(steps_per_mm=[-2, 2, 2, 2, 2, 2],
                                      travel_mm=[[0, 20]] * 6, max_force_kgf=1, max_step_rate=100))
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
                with client.websocket_connect("/events") as socket:
                    self.assertEqual(socket.receive_json()["states"], {})
                    response = client.post("/runs" + mode + "&repeats=2", content=EXAMPLE)
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
                poses = client.get(f"/runs/{first}/files/poses.csv?download=true")
                self.assertIn("attachment", poses.headers["content-disposition"])
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
