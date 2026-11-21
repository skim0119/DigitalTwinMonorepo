import time
import unittest
from pathlib import Path

from control_core import open_manual_controller


class ManualControllerTest(unittest.TestCase):
    def setUp(self):
        self.controller = open_manual_controller(
            Path(__file__).resolve().parents[2] / "pins.json", simulate=True,
        )
        self.addCleanup(self.controller.stop)

    def wait_for(self, predicate):
        deadline = time.monotonic() + 2
        while not predicate():
            self.assertLess(time.monotonic(), deadline)
            time.sleep(0.005)

    def test_fixed_move_and_return_use_native_counts(self):
        self.assertTrue(self.controller.ready())
        self.assertEqual(self.controller.manual_command(2, 400, 10), 0)
        self.wait_for(lambda: self.controller.status()[2][1] == 10)
        self.assertEqual(self.controller.manual_command(2, -400, 0), 10)
        self.wait_for(lambda: self.controller.status()[2][1] == 0)
        time.sleep(0.22)
        self.assertTrue(self.controller.ready())
        self.assertTrue(all(steps == 0 for _, steps, _ in self.controller.status()))

    def test_stop_snapshot_and_expired_hold(self):
        self.controller.manual_command(1, 400)
        self.wait_for(lambda: self.controller.status()[1][1] > 0)
        stopped = self.controller.manual_command(1, 0)
        time.sleep(0.025)
        self.assertEqual(self.controller.status()[1][1], stopped)
        self.controller.manual_command(1, 400)
        time.sleep(0.25)
        count = self.controller.status()[1][1]
        self.assertGreater(count, stopped)
        with self.assertRaisesRegex(RuntimeError, "expired"):
            self.controller.manual_command(1, 400)
        self.assertEqual(self.controller.status()[1][1], count)

    def test_rejects_invalid_rate_and_force_commands(self):
        for rate in (float("nan"), 801):
            with self.assertRaises(ValueError):
                self.controller.manual_command(0, rate)
        with self.assertRaises(ValueError):
            self.controller.manual_command(0, 400, 2**31)
        with self.assertRaises(RuntimeError):
            self.controller.command([0] * 6, False, 0.2)
