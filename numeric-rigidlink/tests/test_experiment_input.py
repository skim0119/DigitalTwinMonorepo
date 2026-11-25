import unittest

import numpy as np

from actuation_csv import parse_experiment


class ExperimentInputTests(unittest.TestCase):
    def test_converts_loads_and_preserves_cycle_time(self):
        header = "body,cycle_elapsed_s," + ",".join(f"tendon_{i}_load_kgf" for i in range(1, 7))
        content = (header + "\narm_base,0.1,1,0,0,0,0,0\narm_tip,0.1,1,0,0,0,0,0\n"
                   "arm_base,0.2,2,0,0,0,0,0\n").encode()
        data = parse_experiment(content, 6)
        np.testing.assert_allclose(data[:, 0], [0, 0.1, 0.2])
        np.testing.assert_allclose(data[:, 1], [9.80665, 9.80665, 19.6133])
        with self.assertRaises(ValueError):
            parse_experiment(content.replace(b"0.2,2,", b"0.2,-2,"), 6)


if __name__ == "__main__":
    unittest.main()
