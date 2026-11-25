"""Jog with an Xbox controller while recording load cells and motion capture."""

import csv
import math
import socket
import time
from datetime import datetime, timezone
from pathlib import Path

from xbox_controller_6motors import main

SAMPLE_INTERVAL_S = 0.1
MAX_LOAD_AGE_S = 1.0


class Recording:
    def add_arguments(self, parser):
        parser.add_argument("--output", type=Path, required=True, help="New CSV file to record")

    def configure(self, parser, args):
        if args.output.exists() or not args.output.parent.is_dir():
            parser.error("--output must be a new file in an existing directory.")
        self.output = args.output

    def start(self, motors):
        import rclpy
        from motion_capture_tracking_interfaces.msg import NamedPoseArray
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions

        self.motors = motors
        self.poses = {}
        self.loads = [(0, None)] * 6
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.socket.connect(("127.0.0.1", 5050))
        self.file = self.output.open("x", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["timestamp_utc", "elapsed_s", "body", "pose_timestamp_ns", "pose_age_s",
                              "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw",
                              *(f"load_{i}_kgf" for i in range(6)),
                              *(f"load_{i}_age_s" for i in range(6)),
                              *(f"motor_{i}_steps" for i in range(6))])
        self.context = rclpy.Context()
        rclpy.init(context=self.context, signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node("xbox_controller_recording", context=self.context)
        self.node.create_subscription(NamedPoseArray, "/poses", self.receive, qos_profile_sensor_data)
        self.started = time.monotonic()
        self.next_sample = self.started
        print(f"Recording to {self.output}; press Ctrl+C to stop.", flush=True)
        return self.pause

    def receive(self, msg):
        now = time.monotonic()
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        for body in msg.poses:
            if body.name.startswith("arm_"):
                p, q = body.pose.position, body.pose.orientation
                self.poses[body.name] = (now, stamp, (p.x, p.y, p.z, q.x, q.y, q.z, q.w))

    def pause(self, seconds):
        import rclpy

        time.sleep(seconds)
        rclpy.spin_once(self.node, timeout_sec=0)
        now = time.monotonic()
        if now < self.next_sample:
            return
        self.next_sample = now + SAMPLE_INTERVAL_S
        self.read_loads()
        self.write_sample(now)

    def read_loads(self):
        """Read available HX711 replies and request the next snapshot."""
        try:
            while True:
                data = self.socket.recv(512).decode("ascii").splitlines()
                readings = []
                for line in data:
                    timestamp, load = line.split()
                    readings.append((int(timestamp), float(load)))
                if len(readings) == 6 and all(stamp >= 0 and math.isfinite(value) for stamp, value in readings):
                    self.loads = readings
        except (BlockingIOError, ConnectionRefusedError):
            pass
        except (UnicodeError, ValueError, IndexError):
            pass
        try:
            self.socket.send(b"hx711-fast")
        except ConnectionRefusedError:
            pass
    def write_sample(self, now):
        stamp = datetime.now(timezone.utc).isoformat()
        ages = [now - ns / 1e9 if ns and 0 <= now - ns / 1e9 else ""
                for ns, _ in self.loads]
        loads = [value if age != "" and age <= MAX_LOAD_AGE_S else ""
                 for age, (_, value) in zip(ages, self.loads)]
        steps = [status[1] for status in self.motors.status()]
        poses = self.poses.items() or [("", (now, "", ("",) * 7))]
        for name, (received, pose_stamp, pose) in poses:
            self.writer.writerow([stamp, now - self.started, name, pose_stamp,
                                  now - received if name else "", *pose, *loads, *ages, *steps])
        self.file.flush()

    def finish(self):
        if hasattr(self, "node"):
            self.node.destroy_node()
        if hasattr(self, "context"):
            self.context.try_shutdown()
        if hasattr(self, "socket"):
            self.socket.close()
        if hasattr(self, "file"):
            self.file.close()
            print(f"Saved recording to {self.output}", flush=True)


if __name__ == "__main__":
    main(Recording())
