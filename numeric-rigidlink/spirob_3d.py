"""Force-controlled 3D rigid-link SpiRob using Newton.

This is a reduced-order tendon-actuated rod, not a calibrated soft continuum model.
Evenly spaced ideal tendons generate generalized joint torques through local moment arms.
"""

from __future__ import annotations

import math
import csv
from pathlib import Path

import numpy as np
import newton
import warp as wp


@wp.kernel
def anneal_body_velocities(velocity: wp.array(dtype=wp.spatial_vector), factor: float):
    i = wp.tid()
    velocity[i] = velocity[i] * factor


# For experiment:
# 21 links, tip radius is 0.7cm, base block radius is about 2.2cm, the full length 27cm, taper angle=3.18 degrees

# Gym-softrobot:
# 24 links, 30cm, root radius 2.3cm, tip radius 1.078cm

MAX_TENDON_FORCE_N = 50.0


class SpiRob:
    def __init__(
        self,
        *,
        links=20,
        length=0.26243,
        root_radius=0.01818,
        tip_radius=0.006725,
        stiffness=1e1,
        density=172.5,
        poisson_ratio=0.5,
        geometry_file=None,
        twist_stiffness=None,
        joint_limit_deg=30.0,
        substeps=64,
        num_tendon=6,
        ramp_time=0.5,
    ):
        if num_tendon < 1:
            raise ValueError("num_tendon must be positive")
        if not math.isfinite(ramp_time) or ramp_time < 0:
            raise ValueError("ramp_time must be finite and nonnegative")
        if not math.isfinite(joint_limit_deg) or not 0 < joint_limit_deg < 90:
            raise ValueError("joint_limit_deg must be between 0 and 90 degrees")
        if not 0 < poisson_ratio < 0.5 + 1e-12:
            raise ValueError("poisson_ratio must be in (0, 0.5]")
        self.poisson_ratio = poisson_ratio
        self.geometry_file = (
            Path(geometry_file)
            if geometry_file
            else Path(__file__).with_name("taper_arm_geometry_joint.csv")
        )
        with self.geometry_file.open(newline="", encoding="utf-8-sig") as stream:
            measurements = list(csv.DictReader(stream))
        links_data = [
            row
            for row in measurements
            if row["Link"].startswith("Link ") or row["Link"] == "Tip"
        ]
        if len(links_data) != links:
            raise ValueError(f"Geometry CSV must provide {links} link rows")
        inner = np.array(
            [float(row["Inner diameter (mm)"].strip()) / 2000 for row in links_data]
        )
        outer = np.array(
            [float(row["Outer diameter (mm)"].strip()) / 2000 for row in links_data]
        )
        if np.any(inner <= 0) or np.any(outer <= inner):
            raise ValueError(
                "Geometry CSV must contain positive inner radii smaller than outer radii"
            )
        self.inner_radii, self.radii = inner, outer
        self.length = float(links_data[-1]["Measured distance"].strip()) / 1000
        self.segment_length = self.length / links
        self.segment_lengths = np.diff(
            np.array(
                [0.0]
                + [float(row["Measured distance"].strip()) / 1000 for row in links_data]
            )
        )
        self.segment_length = float(np.mean(self.segment_lengths))
        self.second_moment = np.pi / 4 * (outer**4 - inner**4)
        self.polar_moment = 2 * self.second_moment
        self.youngs_modulus = np.asarray(stiffness, dtype=float)
        if self.youngs_modulus.ndim == 0:
            self.youngs_modulus = np.full(links - 1, float(self.youngs_modulus))
        if (
            self.youngs_modulus.shape != (links - 1,)
            or not np.isfinite(self.youngs_modulus).all()
            or np.any(self.youngs_modulus <= 0)
        ):
            raise ValueError(
                "stiffness must be positive Young's modulus scalar or one value per joint"
            )
        self.bending_compliance = (
            self.segment_lengths[:-1] / self.second_moment[:-1]
            + self.segment_lengths[1:] / self.second_moment[1:]
        )
        self.torsion_compliance = (
            self.segment_lengths[:-1] / self.polar_moment[:-1]
            + self.segment_lengths[1:] / self.polar_moment[1:]
        )
        self.stiffness = 2 * self.youngs_modulus / self.bending_compliance
        shear_modulus = self.youngs_modulus / (2 * (1 + poisson_ratio))
        twist_stiffness = (
            2 * shear_modulus / self.torsion_compliance
            if twist_stiffness is None
            else twist_stiffness
        )
        twist_stiffness = np.asarray(twist_stiffness, dtype=float)
        if (
            self.stiffness.shape != (links - 1,)
            or not np.isfinite(self.stiffness).all()
            or np.any(self.stiffness <= 0)
        ):
            raise ValueError("stiffness must be positive scalar or one value per joint")
        if twist_stiffness.ndim == 0:
            twist_stiffness = np.full(links - 1, float(twist_stiffness))
        if (
            twist_stiffness.shape != (links - 1,)
            or not np.isfinite(twist_stiffness).all()
            or np.any(twist_stiffness < 0)
        ):
            raise ValueError(
                "twist_stiffness must be finite, nonnegative, and scalar or per-joint"
            )
        self.twist_stiffness = twist_stiffness
        self.n = links
        self.tendon_angles = 2.0 * np.pi * np.arange(num_tendon) / num_tendon
        self.ramp_time = ramp_time
        self.applied_tensions = np.zeros(num_tendon)
        self.ramp_start = self.applied_tensions.copy()
        self.ramp_target = self.applied_tensions.copy()
        self.ramp_elapsed = np.zeros(num_tendon)
        self.joint_limit = math.radians(joint_limit_deg)
        self.sim_time = 0.0
        self.fps = 60.0
        self.frame_dt = 1.0 / self.fps
        self.substeps = substeps
        self.dt = self.frame_dt / substeps

        # Model coordinates in meters, +Z upward. The arm hangs from the origin.
        base_z = 0.0
        builder = newton.ModelBuilder(up_axis=newton.Axis.Z, gravity=(0.0, 0.0, -9.81))
        # The floor grid is visual only; the robot has a fixed base.
        joints = []
        bodies = []
        h = self.segment_length / 2.0
        for i in range(links):
            z = base_z - (i + 0.5) * self.segment_length
            r = float(self.radii[i])
            b = builder.add_link(
                xform=wp.transform(p=wp.vec3(0.0, 0.0, z), q=wp.quat_identity()),
                label=f"segment_{i:02d}",
            )
            mass = (
                density
                * math.pi
                * (r**2 - self.inner_radii[i] ** 2)
                * self.segment_lengths[i]
            )
            cylinder = builder.add_shape_cylinder(
                b,
                radius=r,
                half_height=h,
                cfg=newton.ModelBuilder.ShapeConfig(density=density),
            )
            builder.body_mass[b] = mass
            bodies.append(b)
            if i == 0:
                joints.append(
                    builder.add_joint_fixed(
                        parent=-1,
                        child=b,
                        parent_xform=wp.transform(
                            p=wp.vec3(0.0, 0.0, base_z), q=wp.quat_identity()
                        ),
                        child_xform=wp.transform(
                            p=wp.vec3(0.0, 0.0, h), q=wp.quat_identity()
                        ),
                        label="fixed_base",
                    )
                )
            else:
                Dof = newton.ModelBuilder.JointDofConfig
                # Apply a gradual drive near the nominal limit; retain a wider
                # hard limit as a backup against the XPBD limit/drive switch.
                bound = 2.0 * self.joint_limit
                axes = [
                    Dof(
                        axis=newton.Axis.X,
                        limit_lower=-bound,
                        limit_upper=bound,
                        target_ke=float(self.stiffness[i - 1]),
                        target_kd=0.15,
                        rest_q=0.0,
                    ),
                    Dof(
                        axis=newton.Axis.Y,
                        limit_lower=-bound,
                        limit_upper=bound,
                        target_ke=float(self.stiffness[i - 1]),
                        target_kd=0.15,
                        rest_q=0.0,
                    ),
                    Dof(
                        axis=newton.Axis.Z,
                        limit_lower=-bound,
                        limit_upper=bound,
                        target_ke=float(self.twist_stiffness[i - 1]),
                        target_kd=0.15,
                        rest_q=0.0,
                    ),
                ]
                joints.append(
                    builder.add_joint_d6(
                        parent=bodies[i - 1],
                        child=b,
                        linear_axes=[],
                        angular_axes=axes,
                        parent_xform=wp.transform(
                            p=wp.vec3(0.0, 0.0, -h), q=wp.quat_identity()
                        ),
                        child_xform=wp.transform(
                            p=wp.vec3(0.0, 0.0, h), q=wp.quat_identity()
                        ),
                        label=f"bend_{i:02d}",
                        collision_filter_parent=True,
                    )
                )
        builder.add_articulation(joints, label="SpiRob")
        self.body_indices = np.asarray(bodies, dtype=np.int64)
        builder.color()

        self.model = builder.finalize()
        self.solver = newton.solvers.SolverXPBD(self.model, iterations=24)
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()
        self.collision_pipeline = newton.CollisionPipeline(self.model)
        self.contacts = self.collision_pipeline.contacts()
        newton.eval_fk(
            self.model, self.model.joint_q, self.model.joint_qd, self.state_0
        )
        # Resolve DOF indices dynamically from the joint layout.
        qd_starts = self.model.joint_qd_start.numpy()
        self.dof_indices = np.array(
            [int(qd_starts[j]) + axis for j in joints[1:] for axis in (0, 1)],
            dtype=np.int64,
        )
        q_starts = self.model.joint_q_start.numpy()
        self.q_indices = np.array(
            [int(q_starts[j]) + axis for j in joints[1:] for axis in (0, 1)],
            dtype=np.int64,
        )
        target_starts = self.model.joint_target_q_start.numpy()
        self.target_indices = np.array(
            [int(target_starts[j]) + axis for j in joints[1:] for axis in (0, 1)],
            dtype=np.int64,
        )
        self.plot_q = wp.clone(self.model.joint_q)
        self.plot_qd = wp.clone(self.model.joint_qd)
        self._original_q = self.model.joint_q.numpy().copy()
        self._original_qd = self.model.joint_qd.numpy().copy()

    def reset(self):
        self.state_0.joint_q.assign(self._original_q)
        self.state_0.joint_qd.assign(self._original_qd)
        newton.eval_fk(
            self.model, self.state_0.joint_q, self.state_0.joint_qd, self.state_0
        )
        self.state_1 = self.model.state()
        self.solver = newton.solvers.SolverXPBD(self.model, iterations=24)
        self.sim_time = 0.0
        self.applied_tensions.fill(0.0)
        self.ramp_start.fill(0.0)
        self.ramp_target.fill(0.0)
        self.ramp_elapsed.fill(0.0)

    def set_tendon_efforts(self):
        # Tension in evenly spaced longitudinal tendons. Under the ideal
        # small-angle constant-moment-arm approximation, each tendon produces
        # joint moment T*r. For the downward arm, +X tension bends about -Y;
        # +Y tension bends about +X. These are equivalent *generalized torques*
        # (not literal Newton body-point forces).
        tensions = self.applied_tensions
        tx = np.dot(tensions, np.cos(self.tendon_angles))
        ty = np.dot(tensions, np.sin(self.tendon_angles))
        f = np.zeros(self.model.joint_dof_count, dtype=np.float32)
        for j in range(1, self.n):
            radius = 0.5 * (self.radii[j - 1] + self.radii[j]) * 0.8
            tau_x = radius * ty
            tau_y = -radius * tx
            f[self.dof_indices[2 * (j - 1)]] = tau_x
            f[self.dof_indices[2 * (j - 1) + 1]] = tau_y
        self.control.joint_f.assign(f)

    def update_soft_limits(self):
        """Smoothly bias each joint drive inward near the nominal limit."""
        newton.eval_ik(self.model, self.state_0, self.plot_q, self.plot_qd)
        angles = self.plot_q.numpy()[self.q_indices]
        start = 0.65 * self.joint_limit
        progress = np.clip(
            (np.abs(angles) - start) / (self.joint_limit - start), 0.0, 1.0
        )
        smooth = progress * progress * (3.0 - 2.0 * progress)
        targets = self.control.joint_target_q.numpy()
        targets[self.target_indices] = (
            -np.sign(angles) * (0.2 * self.joint_limit) * smooth
        )
        self.control.joint_target_q.assign(targets)

    def advance_frame(self, targets=None):
        if targets is not None:
            changed = targets != self.ramp_target
            self.ramp_start[changed] = self.applied_tensions[changed]
            self.ramp_target[changed] = targets[changed]
            self.ramp_elapsed[changed] = 0.0
            self.ramp_elapsed += self.frame_dt
            fraction = (
                np.minimum(self.ramp_elapsed / self.ramp_time, 1.0)
                if self.ramp_time
                else 1.0
            )
            self.applied_tensions = self.ramp_start + fraction * (
                self.ramp_target - self.ramp_start
            )
        self.set_tendon_efforts()
        # Translational damping is zero; anneal velocities at 15 / s.
        annealing_factor = math.exp(-10.0 * self.dt)
        for _ in range(self.substeps):
            self.update_soft_limits()
            self.state_0.clear_forces()
            self.collision_pipeline.collide(self.state_0, self.contacts)
            self.solver.step(
                self.state_0, self.state_1, self.control, self.contacts, self.dt
            )
            wp.launch(
                anneal_body_velocities,
                dim=self.model.body_count,
                inputs=[self.state_1.body_qd, annealing_factor],
                device=self.model.device,
            )
            self.state_0, self.state_1 = self.state_1, self.state_0
        self.sim_time += self.frame_dt


def main():
    from run import main as run

    run()


if __name__ == "__main__":
    main()
