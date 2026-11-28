"""Render an off-center orbit driven by a zero-mean rotating force."""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FFMpegWriter
from matplotlib.patches import Circle, FancyArrowPatch

from radial_springs import CIRCULAR_FORCE, DT, RadialSprings, rotating_force_cycle


CYCLES = 5
FPS = 25
INK = "#172b35"
SPRING = "#62847e"
PATH = "#c35327"
CENTER = "#006f75"


def simulate():
    system = RadialSprings()
    forces = rotating_force_cycle()
    steps = len(forces)
    positions = np.zeros((CYCLES * steps + 1, 2))
    cycle_slip = np.zeros(CYCLES)
    for index in range(CYCLES * steps):
        previous_slip = system.slip
        system.step(forces[index % steps])
        positions[index + 1] = system.position
        cycle_slip[index // steps] += np.abs(system.slip - previous_slip).sum()
    orbits = positions[1:].reshape(CYCLES, steps, 2)
    centers = orbits.mean(axis=1)
    radii = np.linalg.norm(orbits - centers[:, None, :], axis=2).mean(axis=1)
    return system.anchors, forces, positions, centers, radii, cycle_slip


def render(output):
    anchors, forces, positions, centers, radii, cycle_slip = simulate()
    steps = len(forces)
    fig = plt.figure(figsize=(12.8, 7.2), facecolor="#f8faf9")
    fig.text(0.07, 0.92, "A rotating force produces a shifted orbit", color=INK,
             fontsize=26, weight="bold")
    fig.text(0.07, 0.87, "Same force magnitude each cycle  ·  Fractional viscoelastic springs with plastic slip",
             color="#425b62", fontsize=13)
    scene = fig.add_axes([0.07, 0.19, 0.39, 0.59])
    zoom = fig.add_axes([0.57, 0.21, 0.36, 0.57])
    for axes in (scene, zoom):
        axes.set_aspect("equal")
        axes.set_facecolor("#f8faf9")
    scene.set(xlim=(-3.4, 3.4), ylim=(-3.4, 3.4), title="Six-spring system")
    scene.set_axis_off()
    scene.scatter(*anchors.T, marker="s", color=INK, s=40)
    springs = [scene.plot([], [], lw=1.55, color=SPRING)[0] for _ in anchors]
    ball = Circle((0, 0), 0.17, facecolor=PATH, edgecolor="white", lw=0.9, zorder=4)
    scene.add_patch(ball)
    arrow = FancyArrowPatch((0, 0), (0.95, 0), arrowstyle="-|>", mutation_scale=16,
                            lw=2.5, color=INK, zorder=5)
    scene.add_patch(arrow)
    scene.text(0.5, -0.025, "Dark arrow shows the applied force", ha="center",
               transform=scene.transAxes, color="#425b62", fontsize=10)

    span = max(np.abs(positions).max(), radii[-1]) * 1.14
    zoom.set(xlim=(-span, span), ylim=(-span, span), xlabel="X displacement",
             ylabel="Y displacement", title="Trajectory close-up")
    zoom.grid(alpha=0.16)
    guide = Circle((0, 0), radii[-1], fill=False, linestyle="--",
                   edgecolor="#788a8f", lw=1.3)
    zoom.add_patch(guide)
    zoom.plot(0, 0, "+", color=INK, ms=9, mew=1.4, zorder=5)
    history, = zoom.plot([], [], lw=1.1, color=SPRING, alpha=0.45)
    trace, = zoom.plot([], [], lw=2.4, color=PATH)
    location, = zoom.plot([], [], "o", color=PATH, ms=6)
    shift, = zoom.plot([], [], "-o", color=CENTER, lw=1.6, ms=6)
    fig.text(0.57, 0.025, "Dashed circle: centered guide of the same mean radius",
             color="#425b62", fontsize=10)

    phase_text = fig.text(0.07, 0.105, "", fontsize=15, color=INK, weight="bold")
    center_text = fig.text(0.57, 0.105, "", fontsize=15, color=CENTER, weight="bold")
    slip_text = fig.text(0.57, 0.065, "", fontsize=11, color="#425b62")
    along = np.linspace(0, 1, 37)
    corrugation = np.zeros_like(along)
    corrugation[2:-2:2] = 0.07
    corrugation[3:-2:2] = -0.07

    def draw(index):
        point = positions[index]
        ball.center = point
        for anchor, spring in zip(anchors, springs):
            segment = anchor - point
            normal = np.array([-segment[1], segment[0]]) / np.linalg.norm(segment)
            vertices = point + np.outer(along, segment) + np.outer(corrugation, normal)
            spring.set_data(*vertices.T)
        direction = forces[index % steps] / CIRCULAR_FORCE
        arrow.set_positions(point, point + 0.95 * direction)

        completed = index // steps
        active_cycle = min(completed, CYCLES - 1)
        cycle_start = active_cycle * steps
        history.set_data(*positions[:cycle_start + 1].T)
        trace.set_data(*positions[cycle_start:index + 1].T)
        location.set_data([point[0]], [point[1]])
        phase_text.set_text(f"Cycle {active_cycle + 1} / {CYCLES}  ·  {index * DT:.1f} s")
        if completed:
            centroid = centers[completed - 1]
            shift.set_data([0, centroid[0]], [0, centroid[1]])
            distance = np.linalg.norm(centroid)
            center_text.set_text(f"Loop center offset  {distance:.3f} units")
            slip_text.set_text(f"Plastic slip in last cycle  {cycle_slip[completed - 1]:.4f} units")
        else:
            center_text.set_text("Measuring the first loop")

    writer = FFMpegWriter(fps=FPS, codec="h264", extra_args=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])
    frames = np.clip(np.rint(np.arange(round((CYCLES * steps * DT + 2) * FPS)) / (FPS * DT)),
                     0, len(positions) - 1).astype(int)
    output.parent.mkdir(parents=True, exist_ok=True)
    with writer.saving(fig, str(output), dpi=100):
        for index in frames:
            draw(index)
            writer.grab_frame()
    plt.close(fig)
    distance = np.linalg.norm(centers[-1])
    print(f"Final center {centers[-1]}, offset {distance:.4f}, mean radius {radii[-1]:.4f}")
    print(f"Plastic slip per cycle: {cycle_slip}")
    print(f"Saved {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("radial_shakedown.mp4"))
    render(parser.parse_args().output)
