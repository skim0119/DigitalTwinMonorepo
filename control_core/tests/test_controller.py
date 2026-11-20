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
                lines = [f"{stamp} {self.force}\n" if self.motor_only in (None, i) else "0 0\n"
                         for i in range(6)]
                self.socket.sendto("".join(lines).encode(), sender)

    def wait_for(self, predicate):
        deadline = time.monotonic() + 2
        while not predicate():
            self.assertLess(time.monotonic(), deadline, "Native controller did not reach expected state")
            time.sleep(0.005)

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
                self.assertIn("tension", str(error))
                return True
            return False

        self.wait_for(faulted)


if __name__ == "__main__":
    unittest.main()
