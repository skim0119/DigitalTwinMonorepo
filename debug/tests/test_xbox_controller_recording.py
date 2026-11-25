import sys
import csv
import io
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from xbox_controller_recording import Recording


class RecordingTest(unittest.TestCase):
    def test_memory_report_uses_current_rss_and_handles_missing_proc(self):
        recording = Recording()
        recording.pose_message_time = None
        with tempfile.TemporaryFile(mode="w+") as recording.file:
            recording.file.write("x" * 1024)
            recording.file.flush()
            with patch.object(Path, "read_text", return_value="Name: python\nVmRSS:\t2048 kB\n"), \
                    patch("builtins.print") as output:
                recording.report_usage(10)
                output.assert_any_call(
                    "Recorder memory (RSS): 2.0 MiB | CSV size: 1.0 KiB", flush=True)
                output.assert_any_call("Poses: waiting for messages on /poses", flush=True)
            with patch.object(Path, "read_text", side_effect=OSError), patch("builtins.print") as output:
                recording.report_usage(10)
                output.assert_any_call(
                    "Recorder memory (RSS): unavailable | CSV size: 1.0 KiB", flush=True)

    def test_matching_poses_are_written_to_csv(self):
        recording = Recording()
        recording.poses = {}
        recording.loads = [(0, None)] * 6
        recording.started = 0
        recording.motors = MagicMock()
        recording.motors.status.return_value = [(0, 0, 0)] * 6
        recording.file = io.StringIO()
        recording.writer = csv.writer(recording.file)
        pose = types.SimpleNamespace(
            position=types.SimpleNamespace(x=1, y=2, z=3),
            orientation=types.SimpleNamespace(x=0, y=0, z=0, w=1))
        msg = types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=10, nanosec=20)),
            poses=[types.SimpleNamespace(name=name, pose=pose) for name in ("arm_base", "arm_tip", "other")])
        with patch("xbox_controller_recording.time.monotonic", return_value=5):
            recording.receive(msg)
        recording.write_sample(5.1)
        rows = list(csv.reader(io.StringIO(recording.file.getvalue())))
        self.assertEqual(len(rows), 1)
        row = rows[0]
        self.assertEqual(len(row), 38)
        for offset in (2, 11):
            self.assertEqual(row[offset], "10000000020")
            self.assertEqual(row[offset + 2:offset + 9], ["1", "2", "3", "0", "0", "0", "1"])
        del recording.poses["arm_base"]
        recording.file = io.StringIO()
        recording.writer = csv.writer(recording.file)
        recording.write_sample(5.2)
        row = next(csv.reader(io.StringIO(recording.file.getvalue())))
        self.assertEqual(row[2:11], [""] * 9)
        self.assertEqual(row[11], "10000000020")

    def test_executor_uses_recording_context_and_is_shut_down(self):
        context = MagicMock()
        node = MagicMock()
        executor = MagicMock()
        ros = types.ModuleType("rclpy")
        ros.Context = MagicMock(return_value=context)
        ros.init = MagicMock()
        ros.create_node = MagicMock(return_value=node)
        modules = {
            "rclpy": ros,
            "rclpy.executors": types.SimpleNamespace(SingleThreadedExecutor=MagicMock(return_value=executor)),
            "rclpy.qos": types.SimpleNamespace(qos_profile_sensor_data=object()),
            "rclpy.signals": types.SimpleNamespace(SignalHandlerOptions=types.SimpleNamespace(NO=0)),
            "motion_capture_tracking_interfaces": types.ModuleType("motion_capture_tracking_interfaces"),
            "motion_capture_tracking_interfaces.msg": types.SimpleNamespace(NamedPoseArray=object()),
        }
        with tempfile.TemporaryDirectory() as directory, patch.dict(sys.modules, modules):
            recording = Recording()
            recording.output = Path(directory) / "recording.csv"
            motors = MagicMock()
            motors.status.return_value = [(0, 0, 0)] * 6
            with patch("xbox_controller_recording.socket.socket"):
                pause = recording.start(motors)
                modules["rclpy.executors"].SingleThreadedExecutor.assert_called_once_with(context=context)
                executor.add_node.assert_called_once_with(node)
                with patch.object(recording, "read_loads"):
                    pause(0)
                executor.spin_once.assert_called_once_with(timeout_sec=0)
                recording.finish()
            executor.shutdown.assert_called_once()
            node.destroy_node.assert_called_once()
            context.try_shutdown.assert_called_once()


if __name__ == "__main__":
    unittest.main()
