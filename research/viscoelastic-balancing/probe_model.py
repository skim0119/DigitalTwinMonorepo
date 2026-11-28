"""Regularized regression of body-frame motion after a complete tendon probe."""

import json
from pathlib import Path

import numpy as np
from scipy.linalg import lstsq
from scipy.spatial.transform import Rotation


def rotation(pose):
    return Rotation.from_quat(pose[3:])


def relative(a, b):
    """T_a^-1 T_b as millimetres and a rotation vector in radians."""
    inv = rotation(a).inv()
    return np.r_[inv.apply(b[:3] - a[:3]), (inv * rotation(b)).as_rotvec()]


def compose(a, change):
    r = rotation(a)
    return np.r_[
        a[:3] + r.apply(change[:3]), (r * Rotation.from_rotvec(change[3:])).as_quat()
    ]


def pose_errors(a, b):
    delta = relative(a, b)
    return np.linalg.norm(delta[:3]), np.degrees(np.linalg.norm(delta[3:]))


def cost(pose, goal):
    mm, degrees = pose_errors(pose, goal)
    return np.hypot(mm / 5, degrees / 8)


POSE_SCALE = np.array([20.0] * 3 + [np.radians(20)] * 3)


def left_jacobian(omega):
    x, y, z = omega
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    angle = np.linalg.norm(omega)
    if angle < 1e-4:
        return np.eye(3) + 0.5 * skew + (skew @ skew) / 6
    return (np.eye(3) + (1 - np.cos(angle)) / angle**2 * skew
            + (angle - np.sin(angle)) / angle**3 * (skew @ skew))


def log_coordinates(change):
    omega = change[3:]
    return np.r_[np.linalg.solve(left_jacobian(omega), change[:3]), omega]


def exp_coordinates(twist):
    omega = twist[3:]
    return np.r_[left_jacobian(omega) @ twist[:3], omega]


def features(state, action):
    scaled = action / 0.25
    pull = np.maximum(scaled, 0)
    release = np.minimum(scaled, 0)
    signed = np.r_[pull, release]
    return np.r_[1.0, state, signed, signed * np.abs(signed),
                 np.outer(signed, state).ravel()]


class ProbeModel:
    def __init__(self, poses=None, memories=None, actions=None, changes=None):
        self.poses, self.memories = poses, memories
        self.actions, self.changes = actions, changes
        if poses is not None:
            self._fit_regression()

    def fit(self, rows):
        self.poses, self.memories, self.actions, self.changes = (
            np.asarray([row[key] for row in rows], dtype=float)
            for key in ("start", "memory", "action", "change")
        )
        self._fit_regression()
        return self

    def _state(self, pose, memory):
        pose_coordinates = log_coordinates(relative(self.anchor, pose)) / POSE_SCALE
        return np.r_[pose_coordinates, (memory - self.memory_center) / 0.1]

    def _fit_regression(self):
        self.anchor = np.r_[self.poses[:, :3].mean(axis=0),
                            Rotation.from_quat(self.poses[:, 3:]).mean().as_quat()]
        self.memory_center = self.memories.mean(axis=0)
        design = np.array([features(self._state(pose, memory), action)
                           for pose, memory, action in zip(self.poses, self.memories, self.actions)])
        target = np.array([log_coordinates(change) for change in self.changes])
        penalty = np.full(design.shape[1], 1.0)
        penalty[0] = 0
        penalty[-12 * (6 + self.memories.shape[1]):] = 10.0  # Shrink state interactions.
        augmented_design = np.vstack((design, np.diag(np.sqrt(penalty))))
        augmented_target = np.vstack((target, np.zeros((design.shape[1], target.shape[1]))))
        self.coefficients = lstsq(augmented_design, augmented_target)[0]
        self.candidates = np.unique(self.actions, axis=0)

    def predictions(self, pose, memory):
        state = self._state(pose, memory)
        for action in self.candidates:
            twist = features(state, action) @ self.coefficients
            yield action, compose(pose, exp_coordinates(twist))

    def choose(self, pose, memory, goal, *, duration=None):
        current_cost = cost(pose, goal)
        best_action = None
        best_improvement = 0.0
        for action, predicted in self.predictions(pose, memory):
            if not np.any(action):
                continue
            improvement = current_cost - cost(predicted, goal)
            if duration is not None:
                improvement /= duration(action)
            if improvement > best_improvement:
                best_action = action
                best_improvement = improvement
        return best_action

    def save(self, path):
        with Path(path).open("wb") as stream:
            np.savez_compressed(
                stream,
                version=1,
                poses=self.poses,
                memories=self.memories,
                actions=self.actions,
                changes=self.changes,
            )

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as data:
            if int(data["version"]) != 1:
                raise ValueError("Unsupported probe model")
            return cls(
                *(data[key] for key in ("poses", "memories", "actions", "changes"))
            )


def export_validation(path, errors, stationary, total):
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    figure = Figure(figsize=(8, 4.5), layout="constrained")
    FigureCanvasAgg(figure)
    if not errors:
        message = ("Collect another block for held-out validation." if total == 0
                   else f"No supported predictions for {total} held-out cycles.")
        figure.text(0.5, 0.5, message, ha="center")
    else:
        figure.suptitle(f"Held-out validation · {len(errors)}/{total} cycles predicted")
        axes = figure.subplots(1, 2, sharey=True)
        percentile = 100 * np.arange(1, len(errors) + 1) / len(errors)
        for index, (ax, label) in enumerate(zip(axes, ("Position error (mm)", "Orientation error (degrees)"))):
            ax.step(np.sort(np.asarray(errors)[:, index]), percentile, where="post", label="Model")
            ax.step(np.sort(np.asarray(stationary)[:, index]), percentile, where="post",
                    linestyle="--", label="Unchanged pose")
            ax.set_xlabel(label)
            ax.grid(alpha=0.25)
        axes[0].set_ylabel("Validation cycles (%)")
        axes[1].legend()
    figure.savefig(path, dpi=150)


def train(directory, path, *, extra_data=()):
    rows = []
    sources = {source.resolve() for root in (directory, *extra_data)
               for source in root.glob("sweep-*.jsonl")}
    for source in sorted(sources):
        with source.open() as stream:
            for line in stream:
                try:
                    row = json.loads(line)
                except json.JSONDecodeError:
                    if not line.endswith("\n"):  # Interrupted final write.
                        break
                    raise
                if row["kind"] == "cycle":
                    rows.append((source, row))
    if not rows:
        raise ValueError("No completed sweep cycles in " + str(directory))
    # Hold out the most recent acquisition block, keeping adjacent cycles together.

    blocks = sorted({(s, r["block"]) for s, r in rows},
                    key=lambda item: (item[0].name, item[0], item[1]))
    errors, stationary = [], []
    total = 0
    model = ProbeModel()

    if len(blocks) > 1:
        held = blocks[-1]
        model.fit([r for s, r in rows if (s, r["block"]) != held])
        total = sum((s, r["block"]) == held for s, r in rows)
        for s, r in rows:
            if (s, r["block"]) != held:
                continue
            start = np.array(r["start"])
            for action, pred in model.predictions(start, np.array(r["memory"])):
                if np.array_equal(action, r["action"]):
                    end = np.array(r["end"])
                    errors.append(pose_errors(pred, end))
                    stationary.append(pose_errors(start, end))
        print(
            f"Held-out coverage: {len(errors)}/{total}"
        )
        if errors:
            print(
                "Median errors (mm, deg), model:",
                np.median(errors, axis=0),
                "unchanged pose:",
                np.median(stationary, axis=0),
            )
    else:
        print("Only one sweep block; held-out evaluation unavailable.")
    model.fit([r for _, r in rows])
    model.save(path)
    print(f"Saved {len(rows)} completed cycles to {path}")
    plot_path = Path(path).with_name(Path(path).stem + "_validation.png")
    export_validation(plot_path, errors, stationary, total)
    print(f"Saved validation plot to {plot_path}")
