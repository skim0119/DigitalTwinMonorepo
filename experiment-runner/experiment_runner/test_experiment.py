"""Contract tests for table validation, durable trial playback, and hardware units."""
import csv
import tempfile
import threading
import time
import unittest
from pathlib import Path

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

    def test_force_trial_can_reopen_away_from_zero_without_api_homing(self):
        config = Calibration.load(
            {"steps_per_mm": [2] * 6, "travel_mm": [[-20, 20]] * 6}, (4.5, 3000),
        )
        bounds_seen = []

        class Controller:
            def ready(self):
                return True

            def status(self):
                return [(0, 3, 0)] * 6

            def stop(self):
                pass

        def open_controller(_path, bounds, **_options):
            bounds_seen.append(bounds)
            return Controller()

        hardware = Hardware(config, open_controller=open_controller)
        force = parse_table(EXAMPLE, "force", "N")
        cancel = threading.Event()
        hardware.open(force, cancel)
        hardware.finish()

        displacement = parse_table(EXAMPLE, "displacement", "mm")
        with self.assertRaisesRegex(RuntimeError, "mechanical zero"):
            hardware.open(displacement, cancel)

        hardware.open(force, cancel)
        self.assertEqual(bounds_seen, [[(-40, 40)] * 6, [(-43, 37)] * 6])
        hardware.finish()
        self.assertEqual(hardware.position_steps, (6,) * 6)

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


class RunnerTest(unittest.TestCase):
    def wait_for(self, predicate):
        deadline = time.monotonic() + 4
        while not predicate():
            self.assertLess(time.monotonic(), deadline, "Experiment failed to reach expected state")
            time.sleep(.02)

    def test_record_flushes_samples_after_controller_stops(self):
        with tempfile.TemporaryDirectory() as folder:
            experiment = Experiments(folder)
            run = {"id": "buffered"}
            directory = Path(folder) / run["id"]
            directory.mkdir()
            paths = [directory / name for name in ("poses.csv", "controls.csv")]
            update = experiment.hardware.update
            updates = 0

            def fail_after_one_sample(*args):
                nonlocal updates
                self.assertEqual([path.stat().st_size for path in paths], [0, 0])
                updates += 1
                if updates == 2:
                    raise RuntimeError("Motion command expired")
                return update(*args)

            def finish():
                self.assertEqual([path.stat().st_size for path in paths], [0, 0])

            experiment.hardware.update = fail_after_one_sample
            experiment.hardware.finish = finish
            with self.assertRaisesRegex(RuntimeError, "Motion command expired"):
                experiment.record(parse_table(EXAMPLE, "force", "N"), run, (), None)
            for path, row_count in zip(paths, (1, 6)):
                with path.open(newline="") as stream:
                    self.assertEqual(len(list(csv.DictReader(stream))), row_count)

    def test_repeats_record_files_and_reload(self):
        with tempfile.TemporaryDirectory() as folder:
            experiment = Experiments(folder, repeat_pause=.01)
            table = parse_table(EXAMPLE, "displacement", "mm")
            runs = experiment.submit(table, EXAMPLE, "Bending", 2, name="baseline")
            self.wait_for(lambda: not experiment.worker.is_alive())
            restored = Experiments(folder)
            for run in runs:
                directory = Path(folder) / run["id"]
                self.assertEqual(restored.get(run["id"])["status"], "complete")
                self.assertTrue(run["id"].endswith("_baseline"))
                self.assertEqual((directory / "actuation.csv").read_bytes(), EXAMPLE)
                with (directory / "poses.csv").open(newline="") as stream:
                    rows = list(csv.DictReader(stream))
                self.assertEqual(tuple(rows[0]), POSE_FIELDS)
                self.assertEqual(rows[0]["body"], "Rigid Body 001")
                self.assertGreater(len(rows), 3)
                self.assertEqual((directory / "diagnostics.png").read_bytes()[:8], b"\x89PNG\r\n\x1a\n")

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

    def test_stop_retains_partial_record_and_cancels_queued_trials(self):
        with tempfile.TemporaryDirectory() as folder:
            experiment = Experiments(folder)
            raw = EXAMPLE.replace(b"0.30", b"2.00")
            table = parse_table(raw, "force", "N")
            first, second = [run["id"] for run in experiment.submit(table, raw, "Bending", 2)]
            self.wait_for(lambda: experiment.get(first)["status"] == "running")
            experiment.stop(first)
            self.wait_for(lambda: not experiment.worker.is_alive())
            self.assertEqual(experiment.get(first)["status"], "cancelled")
            self.assertEqual(experiment.get(second)["status"], "cancelled")
            self.assertTrue((Path(folder) / first / "controls.csv").is_file())


if __name__ == "__main__":
    unittest.main()
