"""Interactive 2D mass with fractional viscoelastic and plastic spring branches."""

from collections import deque

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle


DT = 0.004
MASS = 1.0
ANCHOR_RADIUS = 3.0
CLICK_IMPULSE = 4.5
EQUILIBRIUM_STIFFNESS = 2.2
PLASTIC_STIFFNESS = 10.0
YIELD_EXTENSION = 0.18
FRACTIONAL_ORDER = 0.45
RELAXATION_TIMES = np.geomspace(0.02, 15.0, 14)
MODAL_WEIGHTS = RELAXATION_TIMES ** -FRACTIONAL_ORDER
MODAL_STIFFNESS = 7.5 * MODAL_WEIGHTS / MODAL_WEIGHTS.sum()
MODE_DECAY = np.exp(-DT / RELAXATION_TIMES)
MODE_INPUT = MODAL_STIFFNESS * (-np.expm1(-DT / RELAXATION_TIMES)) * RELAXATION_TIMES / DT


class RadialSprings:
    def __init__(self, yield_extension=YIELD_EXTENSION):
        self.yield_extension = yield_extension
        angles = np.arange(6) * np.pi / 3
        self.anchors = ANCHOR_RADIUS * np.column_stack((np.cos(angles), np.sin(angles)))
        self.reset()

    def reset(self):
        self.position = np.zeros(2)
        self.velocity = np.zeros(2)
        self.extension = np.zeros(6)
        self.slip = np.zeros(6)
        self.modal_force = np.zeros((6, len(RELAXATION_TIMES)))
        self.force = np.zeros(6)

    def update_material(self, extension):
        # The log-spaced Maxwell modes approximate a power-law relaxation kernel.
        increment = extension - self.extension
        self.modal_force *= MODE_DECAY
        self.modal_force += increment[:, None] * MODE_INPUT
        self.extension = extension

        # Return mapping for the spring and slider in the plastic branch.
        self.slip = np.clip(self.slip, extension - self.yield_extension, extension + self.yield_extension)
        self.force = (
            EQUILIBRIUM_STIFFNESS * extension
            + self.modal_force.sum(axis=1)
            + PLASTIC_STIFFNESS * (extension - self.slip)
        )

    def push(self, pointer):
        direction = np.asarray(pointer) - self.position
        length = np.linalg.norm(direction)
        if length > 1e-9:
            self.velocity += CLICK_IMPULSE / MASS * direction / length

    def step(self, external_force=None):
        offsets = self.anchors - self.position
        lengths = np.linalg.norm(offsets, axis=1)
        self.update_material(lengths - ANCHOR_RADIUS)
        axes = offsets / np.maximum(lengths[:, None], 1e-9)
        net_force = self.force @ axes
        if external_force is not None:
            net_force += external_force
        self.velocity += DT / MASS * net_force
        self.position += DT * self.velocity


def build_view(system):
    figure, (scene, response) = plt.subplots(
        1, 2, figsize=(10, 5), layout="constrained", gridspec_kw={"width_ratios": (3, 2)}
    )
    figure.suptitle("Six viscoelastic springs")
    scene.set(xlim=(-3.45, 3.45), ylim=(-3.45, 3.45), aspect="equal",
              title="Click to push the ball  ·  R to reset")
    scene.set_axis_off()
    scene.plot(0, 0, "+", color="0.5", markersize=9)
    scene.scatter(*system.anchors.T, marker="s", color="#304453", s=55, zorder=3)
    for i, anchor in enumerate(system.anchors):
        scene.text(*(anchor * 1.09), str(i + 1), ha="center", va="center", color="#304453")

    springs = [scene.plot([], [], color="#578983", lw=1.6)[0] for _ in range(6)]
    ball = Circle((0, 0), 0.18, color="#d86e32", zorder=4)
    scene.add_patch(ball)
    response.set(xlabel="Extension", ylabel="Axial force")
    response.grid(alpha=0.2)
    response.axhline(0, color="0.7", linewidth=0.7)
    response.axvline(0, color="0.7", linewidth=0.7)
    trace, = response.plot([], [], color="#ad4820", linewidth=1.4, alpha=0.8)
    current, = response.plot([], [], "o", color="#ad4820", markersize=5)
    readout = response.text(0.04, 0.96, "", transform=response.transAxes, va="top")
    selected = 0
    history = deque(maxlen=700)
    along = np.linspace(0, 1, 37)
    zigzag = np.zeros_like(along)
    zigzag[2:-2:2] = 0.07
    zigzag[3:-2:2] = -0.07

    def draw():
        ball.center = system.position
        for i, (anchor, line) in enumerate(zip(system.anchors, springs)):
            span = anchor - system.position
            normal = np.array([-span[1], span[0]]) / max(np.linalg.norm(span), 1e-9)
            path = system.position + np.outer(along, span) + np.outer(zigzag, normal)
            line.set_data(*path.T)
            line.set_color("#ad4820" if i == selected else "#578983")
        point = np.stack((system.extension, system.force))
        if not history or np.any(np.abs(point - history[-1]) > [[0.001], [0.02]]):
            history.append(point)
        curve = np.asarray(history)[:, :, selected]
        trace.set_data(*curve.T)
        current.set_data([system.extension[selected]], [system.force[selected]])
        ext = max(0.2, np.abs(curve[:, 0]).max()) * 1.1
        force = max(0.5, np.abs(curve[:, 1]).max()) * 1.1
        response.set(xlim=(-ext, ext), ylim=(-force, force),
                     title=f"Spring {selected + 1}  ·  Press 1–6 to select")
        displacement = np.linalg.norm(system.position)
        readout.set_text(f"Ball offset  {displacement:.2f}\nPermanent slip  {system.slip[selected]:+.2f}")
        figure.canvas.draw_idle()

    def advance():
        for _ in range(5):
            system.step()
        draw()

    def click(event):
        if event.inaxes is scene and event.button == 1:
            system.push((event.xdata, event.ydata))

    def key(event):
        nonlocal selected
        if event.key == "r":
            system.reset()
            history.clear()
        elif event.key and event.key in tuple("123456"):
            selected = int(event.key) - 1
        draw()

    figure.canvas.mpl_connect("button_press_event", click)
    figure.canvas.mpl_connect("key_press_event", key)
    timer = figure.canvas.new_timer(interval=20)
    timer.add_callback(advance)
    draw()
    return figure, timer


def main():
    figure, timer = build_view(RadialSprings())
    timer.start()
    plt.show()


if __name__ == "__main__":
    main()
