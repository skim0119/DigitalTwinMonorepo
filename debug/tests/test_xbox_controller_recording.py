import sys
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
        with patch.object(Path, "read_text", return_value="Name: python\nVmRSS:\t2048 kB\n"), \
                patch("builtins.print") as output:
            recording.report_memory()
            output.assert_called_once_with("Recorder memory (RSS): 2.0 MiB", flush=True)
        with patch.object(Path, "read_text", side_effect=OSError):
            recording.report_memory()

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
