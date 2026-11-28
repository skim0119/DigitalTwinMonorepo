"""Offline integration checks using synthetic tip motion and load feedback."""
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

from homing import BASE, Rig, run_prediction, run_sweep, sweep_actions
from probe_model import ProbeModel, compose, pose_errors, relative, train


class ProbeTest(unittest.TestCase):
    def test_three_pairs(self):
        actions = sweep_actions()
        self.assertEqual(actions.shape, (55, 6))
        self.assertTrue(np.all(BASE + actions >= 0))
        self.assertAlmostEqual(float((BASE + actions).max()), 0.60)
        paired = [row for row in actions if np.count_nonzero(row) == 2]
        self.assertEqual(len(paired), 36)
        for row in paired:
            self.assertLess(row.min() * row.max(), 0)
        self.assertEqual({tuple(np.flatnonzero(row)) for row in paired},
                         {(0, 5), (1, 3), (2, 4)})

    def test_body_transform(self):
        a = np.r_[[30, 40, -10], Rotation.from_euler('z', 90, degrees=True).as_quat()]
        b = compose(a, [2, 0, 0, 0.2, -0.3, 0.1])
        np.testing.assert_allclose(b[:3], [30, 42, -10])
        np.testing.assert_allclose(relative(a, b), [2, 0, 0, 0.2, -0.3, 0.1], atol=1e-12)
        b[3:] *= -1
        np.testing.assert_allclose(pose_errors(b, compose(a, relative(a, b))), 0, atol=1e-10)

    def test_acquire_fit_reload_and_home(self):
        clock = [0.]
        goal = np.array([0., 0., -100., 0., 0., 0., 1.])
        pose = goal.copy()
        pose[0] = 8
        force = BASE.copy()
        def command(target, mode, ttl):
            force[:] = target
        controller = SimpleNamespace(ready=lambda: True, command=command,
                                     pid_status=lambda: [(v,) for v in force])
        def spin(dt):
            clock[0] += dt
            rig.stamp = clock[0]
            pose[0] -= 2 * max(force[0] - 0.1, 0) * dt
            rig.pose = pose.copy()
        with tempfile.TemporaryDirectory() as folder, patch('homing.time.monotonic', lambda: clock[0]):
            root = Path(folder)
            with (root / 'sweep-test.jsonl').open('w') as stream:
                def log(record):
                    stream.write(json.dumps(record) + "\n")
                    if record["kind"] == "cycle":
                        # Simulate independent probes from the same displaced state.
                        pose[0] = 8
                        rig.memory.fill(0)
                rig = Rig(controller, spin, log, 4.5)
                run_sweep(rig, actions=sweep_actions(), blocks=2, seed=0)
            path = root / 'model.npz'
            train(root, path)
            model = ProbeModel.load(path)
            self.assertEqual(len(model.actions), 110)
            rig.log = lambda r: None
            pose[0] = 8
            run_prediction(rig, model=model, goal=goal, max_probes=12)
            self.assertLessEqual(abs(pose[0]), 5)
            np.testing.assert_allclose(force, BASE)
            rotated_goal = pose.copy()
            rotated_goal[3:] = Rotation.from_euler('z', 9, degrees=True).as_quat()
            _, reached = rig.hold(BASE, "verify", goal=rotated_goal)
            self.assertFalse(reached)
            far = goal.copy()
            far[0] = 1000
            predictions = list(model.predictions(far, rig.memory.ravel()))
            self.assertEqual(len(predictions), 55)
            self.assertTrue(all(np.isfinite(pred).all() for _, pred in predictions))

    def test_delayed_return_frame(self):
        clock = [0.]
        commands = []
        samples = []
        def spin(dt):
            clock[0] += dt
            if not 0.1 < clock[0] < 0.29:
                rig.stamp = clock[0]
        controller = SimpleNamespace(ready=lambda: True,
                                     command=lambda target, *_: commands.append(target),
                                     pid_status=lambda: [(v,) for v in BASE])
        rig = Rig(controller, spin, samples.append, 4.5)
        rig.pose = np.array([0, 0, 0, 0, 0, 0, 1])
        with patch('homing.time.monotonic', lambda: clock[0]):
            rig.hold(BASE, "return", duration=0.25)
            self.assertGreater(clock[0], 0.5)
            self.assertGreater(len(commands), len(samples))
            rig.spin = lambda dt: clock.__setitem__(0, clock[0] + dt)
            with self.assertRaisesRegex(RuntimeError, 'no new valid'):
                rig.hold(BASE, "return")
            recovered_at = clock[0] + 0.3
            controller.ready = lambda: clock[0] >= recovered_at
            rig.spin = spin
            rig.hold(BASE, "return", duration=0.25)
            self.assertGreaterEqual(clock[0], recovered_at + 0.25)
            self.assertIn("load_recovered", [entry["kind"] for entry in samples])
            controller.ready = lambda: False
            rig.spin = spin
            with self.assertRaisesRegex(RuntimeError, 'load-cell'):
                rig.hold(BASE, "return")
