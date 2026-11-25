import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


class RunTests(unittest.TestCase):
    def test_constitutive_selection_and_saved_configuration(self):
        with tempfile.TemporaryDirectory() as output:
            for law, flags in (("linear", []), ("maxwell", ["-C", "maxwell", "--branch-stiffness", "0.3"]),
                               ("fractional", ["--constitutive", "fractional", "--alpha", "0.4"])):
                if law == "maxwell":
                    flags += ["--relaxation-time", "2"]
                subprocess.run([sys.executable, "run.py", *flags, "--duration", "0.01", "--output", output],
                               check=True, stdout=subprocess.DEVNULL)
                config = json.loads((Path(output) / f"{law}.json").read_text())
                self.assertEqual(config["constitutive"], law)
                self.assertEqual(config["model_parameters"]["links"], 20)
                self.assertEqual(config["model_parameters"]["poisson_ratio"], 0.5)
                self.assertEqual(config["actuation"]["units"], "N")
                self.assertTrue((Path(output) / f"{law}.csv").exists())
                if law == "maxwell":
                    self.assertEqual(config["material_parameters"]["branch_stiffness"], [0.3])
                if law == "fractional":
                    self.assertEqual(config["material_parameters"]["alpha"], 0.4)


if __name__ == "__main__":
    unittest.main()
