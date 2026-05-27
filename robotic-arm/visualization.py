from __future__ import annotations

from typing import List, Sequence, Tuple

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.lines import Line2D


class MatplotlibVisualizer:
    """Matplotlib view with one arm plot and per-joint response plots."""

    def __init__(
        self,
        joint_count: int,
        history_window_s: float = 12.0,
        arm_xlim: Tuple[float, float] = (-2.5, 2.5),
        arm_ylim: Tuple[float, float] = (-2.5, 2.5),
    ) -> None:
        if joint_count <= 0:
            raise ValueError("joint_count must be positive.")

        self.joint_count = int(joint_count)
        self.history_window_s = float(history_window_s)
        self.closed = False

        self.fig, axes = plt.subplots(
            nrows=self.joint_count + 1,
            ncols=1,
            figsize=(9, 3.0 + 2.2 * self.joint_count),
            squeeze=False,
        )
        flat_axes = [axis for row in axes for axis in row]
        self.arm_axis: Axes = flat_axes[0]
        self.joint_axes: List[Axes] = flat_axes[1:]

        self.arm_axis.set_title("Robotic Arm State")
        self.arm_axis.set_aspect("equal", adjustable="box")
        self.arm_axis.set_xlim(*arm_xlim)
        self.arm_axis.set_ylim(*arm_ylim)
        self.arm_axis.grid(True, alpha=0.3)
        (self.arm_line,) = self.arm_axis.plot([], [], "o-", lw=3, color="#3b82f6")
        self.status_text = self.arm_axis.text(
            0.02,
            0.95,
            "",
            transform=self.arm_axis.transAxes,
            va="top",
            fontsize=10,
        )

        self.measured_lines: List[Line2D] = []
        self.target_lines: List[Line2D] = []
        for index, axis in enumerate(self.joint_axes):
            axis.set_title(f"Joint {index + 1} Angle Response")
            axis.set_ylabel("Angle [rad]")
            axis.grid(True, alpha=0.3)
            (measured_line,) = axis.plot([], [], lw=2, label="measured")
            (target_line,) = axis.plot([], [], "--", lw=1.8, label="target")
            axis.legend(loc="upper right")
            self.measured_lines.append(measured_line)
            self.target_lines.append(target_line)

        self.joint_axes[-1].set_xlabel("Time [s]")
        self.fig.tight_layout()
        self.fig.canvas.mpl_connect("close_event", self._on_close)

    def _on_close(self, _event: object) -> None:
        self.closed = True

    def update(
        self,
        time_values: Sequence[float],
        measured_angles_per_joint: Sequence[Sequence[float]],
        target_angles_per_joint: Sequence[Sequence[float]],
        arm_points: Sequence[Tuple[float, float]],
        controller_name: str,
        noise_enabled: bool,
    ) -> tuple[Line2D, ...]:
        xs = [point[0] for point in arm_points]
        ys = [point[1] for point in arm_points]
        self.arm_line.set_data(xs, ys)

        current_time = time_values[-1] if time_values else 0.0
        noise_label = "ON" if noise_enabled else "OFF"
        self.status_text.set_text(
            f"t={current_time:5.2f}s | controller={controller_name} | sensor noise={noise_label}"
        )

        xmin = max(0.0, current_time - self.history_window_s)
        xmax = max(self.history_window_s, current_time + 0.02)
        artists: List[Line2D] = [self.arm_line]

        for idx in range(self.joint_count):
            measured = measured_angles_per_joint[idx]
            target = target_angles_per_joint[idx]
            measured_line = self.measured_lines[idx]
            target_line = self.target_lines[idx]

            measured_line.set_data(time_values, measured)
            target_line.set_data(time_values, target)

            axis = self.joint_axes[idx]
            axis.set_xlim(xmin, xmax)
            axis.relim()
            axis.autoscale_view(scaley=True)
            artists.extend([measured_line, target_line])

        return tuple(artists)
