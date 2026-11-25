"""Run the rigid-link models without a viewer and save their response."""

import argparse
import csv
import inspect
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt

import numpy as np
import warp as wp

from actuation_csv import parse_actuation, parse_experiment


def main():
    parser = argparse.ArgumentParser(description="Run the SpiRob simulation")
    parser.add_argument(
        "-C",
        "--constitutive",
        choices=("linear", "maxwell", "fractional"),
        default="linear",
    )
    parser.add_argument(
        "--device", default="cpu", help="Warp device, e.g. cpu or cuda:0"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="duration in seconds (default: input CSV duration, or 2)",
    )
    parser.add_argument(
        "--input",
        "--actuation",
        dest="input",
        type=Path,
        help="actuation CSV or postprocessed experimental cycle CSV",
    )
    parser.add_argument("--num-tendon", type=int, default=6)
    parser.add_argument("--ramp-time", type=float, default=0.5)
    parser.add_argument("--twist-stiffness", type=float, default=None)
    parser.add_argument(
        "--output", type=Path, default=Path(__file__).resolve().parent / "output"
    )
    parser.add_argument(
        "--equilibrium-stiffness", type=float, default=1e6, help="Young's modulus (Pa)"
    )
    parser.add_argument(
        "--branch-stiffness",
        type=float,
        nargs="+",
        default=[1e5, 1e5, 1e5],
        help="Maxwell branch Young's moduli (Pa)",
    )
    parser.add_argument(
        "--relaxation-time", type=float, nargs="+", default=[0.1, 1.0, 10.0]
    )
    parser.add_argument("--fractional-coefficient", type=float, default=0.01)
    parser.add_argument("--alpha", type=float, default=0.3)
    parser.add_argument("--history-seconds", type=float, default=2.0)
    args = parser.parse_args()
    model = args.constitutive
    if args.num_tendon < 1 or not math.isfinite(args.ramp_time) or args.ramp_time < 0:
        parser.error("num-tendon must be positive and ramp-time finite and nonnegative")
    if args.duration is not None and (
        not math.isfinite(args.duration) or args.duration <= 0
    ):
        parser.error("duration must be finite and positive")
    if args.input:
        try:
            content = args.input.read_bytes()
            header = content.splitlines()[0].decode("utf-8-sig") if content else ""
            data = (
                parse_experiment(content, args.num_tendon)
                if "cycle_elapsed_s" in header
                else parse_actuation(content, args.num_tendon, 50)
            )
        except (OSError, UnicodeDecodeError, ValueError) as exc:
            parser.error(str(exc))
        duration = args.duration if args.duration is not None else data[-1, 0]
    else:
        duration = args.duration if args.duration is not None else 2.0
        target = np.zeros(args.num_tendon)
        target[0] = 1.0
        data = np.array(
            [[0, *np.zeros(args.num_tendon)], [max(args.ramp_time, 1e-9), *target]]
        )

    def values_at(t):
        return np.array(
            [np.interp(t, data[:, 0], data[:, i]) for i in range(1, data.shape[1])]
        )

    wp.set_device(args.device)
    options = dict(
        num_tendon=args.num_tendon,
        ramp_time=args.ramp_time,
        twist_stiffness=args.twist_stiffness,
    )
    try:
        from constitutive_models import (
            ViscoelasticSpiRob,
            Linear,
            GeneralizedMaxwell,
            FractionalKelvinVoigt,
        )

        if (
            not math.isfinite(args.equilibrium_stiffness)
            or args.equilibrium_stiffness <= 0
        ):
            raise ValueError("equilibrium stiffness must be finite and positive")
        from spirob_3d import SpiRob

        model_class = SpiRob
        options["stiffness"] = args.equilibrium_stiffness
        if model == "linear":
            material = {}
            factory = Linear
        elif model == "maxwell":
            material = {
                "branch_stiffness": args.branch_stiffness,
                "relaxation_time": args.relaxation_time,
            }
            probe = SpiRob(stiffness=args.equilibrium_stiffness)
            branch_stiffness = (
                np.asarray(args.branch_stiffness)[:, None]
                * 2
                / probe.bending_compliance[None, :]
            )
            factory = lambda shape, dt: GeneralizedMaxwell(
                shape, dt, branch_stiffness, args.relaxation_time
            )
        else:
            material = {
                "coefficient": args.fractional_coefficient,
                "alpha": args.alpha,
                "history_seconds": args.history_seconds,
            }
            probe = SpiRob(stiffness=args.equilibrium_stiffness)
            fractional_coefficient = (
                args.fractional_coefficient * 2 / probe.bending_compliance
            )
            factory = lambda shape, dt: FractionalKelvinVoigt(
                shape, dt, fractional_coefficient, args.alpha, args.history_seconds
            )
        robot = ViscoelasticSpiRob(**options, material_factory=factory)
    except ValueError as exc:
        parser.error(str(exc))

    rows, shapes = [], []

    def record():
        poses = robot.state_0.body_q.numpy()[robot.body_indices].copy()
        q, w = poses[-1, 3:6], poses[-1, 6]
        local = np.array([0.0, 0.0, -robot.segment_length / 2])
        tip = poses[-1, :3] + local + 2 * np.cross(q, np.cross(q, local) + w * local)
        rows.append(
            [
                robot.sim_time,
                *tip,
                *robot.applied_tensions,
                *values_at(robot.sim_time),
                *poses.ravel(),
            ]
        )
        shapes.append(np.vstack(([0.0, 0.0, 0.0], poses[:, :3], tip)))

    record()
    for _ in range(math.ceil(duration / robot.frame_dt)):
        robot.applied_tensions = values_at(robot.sim_time + robot.frame_dt)
        robot.advance_frame()
        record()
    args.output.mkdir(parents=True, exist_ok=True)
    prefix = args.output / model
    parameters = inspect.signature(model_class).bind(**options)
    parameters.apply_defaults()
    geometry_and_solver = dict(parameters.arguments)
    if geometry_and_solver["twist_stiffness"] is None:
        geometry_and_solver["twist_stiffness"] = geometry_and_solver["stiffness"]
    configuration = {
        "constitutive": args.constitutive,
        "control": "force",
        "device": args.device,
        "model_parameters": geometry_and_solver,
        "material_parameters": material,
        "elasticity": {
            "youngs_modulus_pa": robot.youngs_modulus.tolist(),
            "poisson_ratio": robot.poisson_ratio,
            "second_moment_m4": robot.second_moment.tolist(),
            "joint_bending_stiffness_nm_rad": robot.stiffness.tolist(),
            "joint_twist_stiffness_nm_rad": np.asarray(robot.twist_stiffness).tolist(),
        },
        "solver": {
            "method": "XPBD",
            "iterations": 24,
            "frame_dt_s": robot.frame_dt,
            "substep_dt_s": robot.dt,
            "gravity_m_s2": [0.0, 0.0, -9.81],
            "velocity_annealing_rate_s_inv": 10.0,
            "joint_damping": 0.15,
            "hard_joint_limit_multiplier": 2.0,
            "soft_limit_start_fraction": 0.65,
            "soft_limit_target_fraction": 0.2,
            "tendon_moment_arm_radius_fraction": 0.8,
        },
        "requested_duration_s": duration,
        "actual_duration_s": robot.sim_time,
        "actuation": {
            "units": "N",
            "source": str(args.input) if args.input else "default",
            "time_and_tendon_values": data.tolist(),
        },
    }
    with prefix.with_suffix(".json").open("w") as f:
        json.dump(configuration, f, indent=2, allow_nan=False)
        f.write("\n")
    header = ["time_s", "tip_x_m", "tip_y_m", "tip_z_m"]
    header += [f"tension_{i + 1}_N" for i in range(args.num_tendon)]
    header += [f"command_{i + 1}_N" for i in range(args.num_tendon)]
    header += [
        f"link_{i:02d}_{field}"
        for i in range(robot.n)
        for field in ("x_m", "y_m", "z_m", "qx", "qy", "qz", "qw")
    ]
    with prefix.with_suffix(".csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)
    result = np.asarray(rows)
    fig, axes = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    axes[0].plot(result[:, 0], result[:, 1:4])
    axes[0].legend(["x", "y", "z"])
    axes[0].set_ylabel("Tip position (m)")
    axes[1].plot(result[:, 0], result[:, 4 : 4 + args.num_tendon])
    axes[1].set_ylabel("Tendon tension (N)")
    axes[1].set_xlabel("Time (s)")
    fig.tight_layout()
    fig.savefig(prefix.with_name(prefix.name + "_response.png"), dpi=150)
    plt.close(fig)
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(projection="3d")
    for i in np.unique(np.linspace(0, len(shapes) - 1, min(5, len(shapes)), dtype=int)):
        shape = shapes[i]
        ax.plot(*shape.T, label=f"{result[i, 0]:.2f} s")
    ax.set(xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
    ax.set_box_aspect((1, 1, 1))
    ax.legend()
    fig.tight_layout()
    fig.savefig(prefix.with_name(prefix.name + "_shapes.png"), dpi=150)
    plt.close(fig)
    print(f"Saved {prefix}.csv and plots in {args.output}")


if __name__ == "__main__":
    main()
