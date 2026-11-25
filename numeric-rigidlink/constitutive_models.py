"""Rotational viscoelasticity for the reduced-order force-controlled SpiRob.

The notes describe rod couples B*kappa. For a link length L and joint angle
theta, kappa ~= theta/L, so a rod bending stiffness B maps to a joint spring
constant B/L in N m/rad. The constants here are joint constants, not fitted TPU
material properties. The original XPBD drive supplies the equilibrium spring;
these models add only the history-dependent generalized bending torques.
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np
import newton

try:
    from .spirob_3d import SpiRob
except ImportError:
    from spirob_3d import SpiRob


class Linear:
    """The equilibrium spring is supplied by the XPBD joint drive."""

    def __init__(self, shape, dt):
        self.moment = np.zeros(shape)

    def reset(self):
        pass

    def update(self, angles):
        return self.moment


class GeneralizedMaxwell:
    """Exact linear-ramp update of parallel Maxwell branches per joint axis."""

    def __init__(self, shape, dt, stiffnesses, relaxation_times):
        self.shape = shape
        self.dt = dt
        self.stiffnesses = np.asarray(stiffnesses, dtype=float)
        self.relaxation_times = np.asarray(relaxation_times, dtype=float)
        if (
            dt <= 0
            or not math.isfinite(dt)
            or self.stiffnesses.ndim not in (1, 2)
            or not len(self.stiffnesses)
            or self.stiffnesses.shape[0] != len(self.relaxation_times)
            or not np.isfinite(self.stiffnesses).all()
            or np.any(self.stiffnesses < 0)
            or not np.isfinite(self.relaxation_times).all()
            or np.any(self.relaxation_times <= 0)
        ):
            raise ValueError(
                "Maxwell branches need matching nonnegative stiffnesses and positive times"
            )
        a = np.exp(-dt / self.relaxation_times)
        self.decay = a[:, None, None]
        gains = self.stiffnesses * (self.relaxation_times * (-np.expm1(-dt / self.relaxation_times)) / dt).reshape((-1,) + (1,) * (self.stiffnesses.ndim - 1))
        self.gain = gains.reshape((len(gains),) + gains.shape[1:] + (1,) * (2 - (gains.ndim - 1)))
        self.reset()

    def reset(self):
        self.previous = np.zeros(self.shape)
        self.branches = np.zeros((len(self.stiffnesses), *self.shape))

    def update(self, angles):
        angles = np.asarray(angles)
        self.branches = self.decay * self.branches + self.gain * (
            angles - self.previous
        )
        self.previous = angles.copy()
        return self.branches.sum(axis=0)


class FractionalKelvinVoigt:
    """Truncated Grünwald–Letnikov springpot, referenced to zero initial angle."""

    def __init__(self, shape, dt, coefficient, alpha, history_seconds):
        if not (
            math.isfinite(dt)
            and dt > 0
            and np.isfinite(coefficient).all()
            and np.all(np.asarray(coefficient) >= 0)
            and math.isfinite(alpha)
            and 0 < alpha < 1
            and math.isfinite(history_seconds)
            and history_seconds >= dt
        ):
            raise ValueError(
                "fractional model requires C >= 0, 0 < alpha < 1, and history >= dt"
            )
        self.shape = shape
        self.capacity = math.ceil(history_seconds / dt) + 1
        self.factor = np.asarray(coefficient, dtype=float) * dt ** (-alpha)
        self.weights = np.empty(self.capacity)
        self.weights[0] = 1.0
        for j in range(1, self.capacity):
            self.weights[j] = self.weights[j - 1] * (1.0 - (alpha + 1.0) / j)
        self.reset()

    def reset(self):
        self.history = deque(maxlen=self.capacity)

    def update(self, angles):
        self.history.appendleft(np.array(angles, copy=True))
        history = np.asarray(self.history)
        derivative = np.tensordot(self.weights[: len(history)], history, axes=(0, 0))
        factor = np.asarray(self.factor)
        return factor[:, None] * derivative if factor.ndim else factor * derivative


class ViscoelasticSpiRob(SpiRob):
    def __init__(self, *, material_factory, **kwargs):
        super().__init__(**kwargs)
        self.material = material_factory((self.n - 1, 2), self.dt)
        self._tendon_forces = np.zeros(self.model.joint_dof_count, dtype=np.float32)

    def reset(self):
        super().reset()
        self.material.reset()
        self._tendon_forces.fill(0.0)

    def set_tendon_efforts(self):
        super().set_tendon_efforts()
        self._tendon_forces = self.control.joint_f.numpy().copy()

    def update_soft_limits(self):
        super().update_soft_limits()
        # The base class recovers D6 joint angles via eval_ik. Apply the
        # constitutive history torque opposite to positive bending rotation.
        angles = self.plot_q.numpy()[self.q_indices].reshape(-1, 2)
        viscous_moment = self.material.update(angles)
        efforts = self._tendon_forces.copy()
        efforts[self.dof_indices] -= viscous_moment.ravel()
        self.control.joint_f.assign(efforts)
