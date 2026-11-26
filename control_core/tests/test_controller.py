import json
import socket
import sys
import threading
import time
import unittest
from pathlib import Path

from control_core import open_controller


class NativeControllerTest(unittest.TestCase):
    def setUp(self):
        self.force = 0.05
        self.publish = True
        self.motor_only = None
        self.missing_motor = None
        self.streaming = threading.Event()
        self.streaming.set()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(self.socket.close)
        self.socket.bind(("127.0.0.1", 0))
        self.socket.settimeout(0.05)
        worker = threading.Thread(target=self.serve)
        worker.start()
        self.addCleanup(worker.join, 1)
        self.addCleanup(self.streaming.clear)
        self.controller = open_controller(
            Path(__file__).resolve().parents[2] / "pins.json",
            [(-3, 3)] * 6, port=self.socket.getsockname()[1],
            max_force=1, max_rate=150, simulate=True,
        )
        self.addCleanup(self.controller.stop)
        self.wait_for(self.controller.ready)

    def serve(self):
        # Match the C++ steady_clock used by the streamer on each platform.
        clock = time.CLOCK_MONOTONIC_RAW if sys.platform == "darwin" else time.CLOCK_MONOTONIC
        stamp = 0
        while self.streaming.is_set():
            try:
                request, sender = self.socket.recvfrom(512)
            except socket.timeout:
                continue
            if request == b"hx711-fast":
                now = time.clock_gettime_ns(clock)
                if self.publish and now - stamp >= 10_000_000:
                    stamp = now
                lines = [f"{stamp} {self.force}\n" if self.motor_only in (None, i) and i != self.missing_motor else "0 0\n"
                         for i in range(6)]
                self.socket.sendto("".join(lines).encode(), sender)

    def wait_for(self, predicate):
        deadline = time.monotonic() + 2
        while not predicate():
            self.assertLess(time.monotonic(), deadline, "Native controller did not reach expected state")
            time.sleep(0.005)

    def test_missing_force_target_idles_only_that_motor(self):
        self.controller.command([0.2, None, None, None, None, None], True, 0.25)
        self.wait_for(lambda: self.controller.status()[0][1] == 3)
        self.assertEqual(self.controller.status()[1][1:], (0, 0))
        self.controller.command([None, 0.2, None, None, None, None], True, 0.25)
        self.wait_for(lambda: self.controller.status()[1][1] == 3)
        self.assertEqual(self.controller.status()[0][1:], (3, 0))
        self.controller.command([0, None, None, None, None, None], True, 0.25)
        self.wait_for(lambda: self.controller.status()[0][1] < 3)

    def test_force_all_channels_and_position_bounds(self):
        self.controller.command([0.2] * 6, True, 0.2)
        self.wait_for(lambda: all(steps == 3 for _, steps, _ in self.controller.status()))
        self.wait_for(lambda: all(speed == 0 for _, _, speed in self.controller.status()))

    def test_expired_command_stops_and_cannot_restart(self):
        self.controller.command([3] * 6, False, 0.03)
        time.sleep(0.05)
        with self.assertRaisesRegex(RuntimeError, "expired"):
            self.controller.command([0] * 6, False, 0.1)
        self.wait_for(lambda: all(speed == 0 for _, _, speed in self.controller.status()))

    def test_duplicate_timestamps_do_not_refresh_sensor_deadline(self):
        self.publish = False

        def refresh_until_stale():
            try:
                self.controller.command([0.2] * 6, True, 0.2)
            except RuntimeError as error:
                self.assertIn("stale", str(error))
                return True
            return False

        self.wait_for(refresh_until_stale)
        self.wait_for(lambda: all(speed == 0 for _, _, speed in self.controller.status()))

    def test_invalid_sample_pauses_until_two_fresh_conversions(self):
        self.controller.command([0.08] * 6, True, 0.25)
        self.wait_for(lambda: self.controller.status()[0][2] > 0)
        self.motor_only = 1
        self.wait_for(lambda: self.controller.status()[0][2] == 0)
        paused_steps = self.controller.status()[0][1]

        self.publish = False
        self.motor_only = None
        self.controller.command([0.08] * 6, True, 0.25)
        time.sleep(0.04)
        self.assertEqual(self.controller.status()[0][1:], (paused_steps, 0))

        self.publish = True
        self.wait_for(lambda: self.controller.status()[0][2] > 0)

    def test_single_motor_ignores_other_sensors_and_stops_at_tightening_limit(self):
        self.controller.stop()
        self.motor_only = 2
        self.controller = open_controller(
            Path(__file__).resolve().parents[2] / "pins.json",
            [(-10, 10)] * 6, port=self.socket.getsockname()[1],
            max_force=1, max_rate=150, simulate=True, motor=2, max_tightening_steps=2,
        )
        self.addCleanup(self.controller.stop)
        self.wait_for(self.controller.ready)
        self.controller.command([0.2] * 6, True, 0.2)
        self.wait_for(lambda: self.controller.tightening_steps(2) == 2)
        self.assertEqual([step for _, step, _ in self.controller.status()], [0, 0, 2, 0, 0, 0])
        with self.assertRaisesRegex(RuntimeError, "Tightening STEP limit"):
            self.controller.command([0.2] * 6, True, 0.2)

    def test_skipped_motor_ignores_missing_sensor_and_never_moves(self):
        self.controller.stop()
        self.missing_motor = 2
        self.controller = open_controller(
            Path(__file__).resolve().parents[2] / "pins.json",
            [(-10, 10)] * 6, port=self.socket.getsockname()[1],
            max_force=1, max_rate=150, simulate=True, skip_motor=2,
        )
        self.addCleanup(self.controller.stop)
        self.wait_for(self.controller.ready)
        self.controller.command([0.2] * 6, True, 0.2)
        self.wait_for(lambda: all(self.controller.tightening_steps(i) > 0
                                  for i in (0, 1, 3, 4, 5)))
        self.assertEqual(self.controller.status()[2], (0, 0, 0))

    def test_live_gain_edits_take_effect_within_one_session(self):
        self.controller.stop()
        self.controller = open_controller(
            Path(__file__).resolve().parents[2] / "pins.json",
            [(-100, 100)] * 6, port=self.socket.getsockname()[1],
            max_force=1, max_rate=150, simulate=True, motor=2,
        )
        self.addCleanup(self.controller.stop)
        self.wait_for(self.controller.ready)
        self.controller.command([0.2] * 6, True, 0.2, gains=(0, 0, 0))
        time.sleep(0.04)
        self.assertEqual(self.controller.tightening_steps(2), 0)
        self.controller.command([0.2] * 6, True, 0.2, gains=(550, 0, 0))
        self.wait_for(lambda: self.controller.tightening_steps(2) >= 1)
        self.controller.command([0.2] * 6, True, 0.2, gains=(0, 0, 0))
        self.wait_for(lambda: self.controller.status()[2][2] == 0)
        self.assertEqual(self.controller.status()[1], (0, 0, 0))

    def test_over_tension_uses_unfiltered_conversion(self):
        self.controller.command([0.2] * 6, True, 0.2)
        self.force = 1.1

        def faulted():
            try:
                self.controller.ready()
            except RuntimeError as error:
                self.assertIn("motor 0 (tendon_1): unfiltered 1.1 kgf, limit 1 kgf", str(error))
                return True
            return False

        self.wait_for(faulted)

    def test_target_updates_keep_integral_without_derivative_kick(self):
        self.controller.stop()
        self.controller = open_controller(
            Path(__file__).resolve().parents[2] / "pins.json",
            [(-100, 100)] * 6, port=self.socket.getsockname()[1],
            max_force=1, max_rate=150, simulate=True, motor=2,
        )
        self.addCleanup(self.controller.stop)
        self.wait_for(self.controller.ready)
        self.controller.command([0.2] * 6, True, 0.25, gains=(0, 1000, 1000))
        self.wait_for(lambda: self.controller.status()[2][2] >= 8)
        for target in (0.21, 0.22, 0.23):
            self.controller.command([target] * 6, True, 0.25)
            deadline = time.monotonic() + 0.025
            while time.monotonic() < deadline:
                speed = self.controller.status()[2][2]
                self.assertGreaterEqual(speed, 8)
                self.assertLess(speed, 100)
                time.sleep(0.002)

    def test_output_filter_and_term_snapshot(self):
        self.controller.command([0.2] * 6, True, 0.25)
        self.wait_for(lambda: self.controller.pid_status()[0][-1] > 0)
        _, _, p, i, d, ff, requested, filtered = self.controller.pid_status()[0]
        self.assertAlmostEqual(p + i + d + ff, requested)
        self.assertGreater(filtered, 0)
        self.assertLess(filtered, requested)

    def test_feedforward_ramps_and_stops_immediately_on_force_limit(self):
        self.controller.stop()
        config = json.loads((Path(__file__).resolve().parents[2] / "pins.json").read_text())
        config["pid_max_acceleration_steps_s2"] = 600
        config["motors"][2]["pid"] = dict(kp=0, ki=0, kd=0, kff=100)
        self.controller = open_controller(
            config, [(-100, 100)] * 6, port=self.socket.getsockname()[1],
            max_force=1, max_rate=150, simulate=True, motor=2,
        )
        self.addCleanup(self.controller.stop)
        self.wait_for(self.controller.ready)
        self.controller.command([0.05] * 6, True, 0.25, target_rates=[1.0] * 6)
        self.wait_for(lambda: self.controller.status()[2][2] > 0)
        self.assertLess(self.controller.status()[2][2], 50)
        self.wait_for(lambda: self.controller.status()[2][2] > 50)
        self.controller.command([0.05] * 6, True, 0.25, target_rates=[-1.0] * 6)
        self.wait_for(lambda: self.controller.status()[2][2] == 0)
        self.wait_for(lambda: self.controller.status()[2][2] < 0)
        self.force = 1.1
        self.wait_for(lambda: self.controller.status()[2][2] == 0)
        with self.assertRaisesRegex(RuntimeError, "tension"):
            self.controller.ready()


if __name__ == "__main__":
    unittest.main()
