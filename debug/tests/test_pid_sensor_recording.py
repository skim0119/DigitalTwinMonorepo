import csv
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PID_multi import SensorRecorder


class SensorRecorderTest(unittest.TestCase):
    def test_records_all_loads_and_marks_stale_or_missing_readings(self):
        with tempfile.TemporaryDirectory() as directory, patch("PID_multi.socket.socket") as make_socket:
            sock = make_socket.return_value
            sock.recv.side_effect = [
                b"9000000000 1\n9000000000 2\n0 0\n8000000000 4\n9000000000 5\n9000000000 6\n",
                BlockingIOError(),
            ]
            path = Path(directory) / "sensors.csv"
            recorder = SensorRecorder(path, 5050)
            recorder.sample(0.5, 10.0, [(0, i, 0) for i in range(6)])
            recorder.close()
            with path.open(newline="") as file:
                row, = csv.DictReader(file)
            self.assertEqual([row[f"load_{i}_kgf"] for i in range(6)],
                             ["1.0", "2.0", "", "", "5.0", "6.0"])
            self.assertEqual(row["load_3_age_s"], "2.0")
            self.assertEqual([row[f"motor_{i}_commanded_steps"] for i in range(6)],
                             [str(i) for i in range(6)])
            sock.send.assert_called_once_with(b"hx711-fast")


if __name__ == "__main__":
    unittest.main()
