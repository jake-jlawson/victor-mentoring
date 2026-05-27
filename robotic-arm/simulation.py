from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Sequence

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from controllers.base import ControllerBase, JointState, JointTarget
from controllers.feedforward import FeedforwardController
from controllers.open_loop import OpenLoopController, SinusoidalTorqueProfile
from controllers.pid import PIDController
from controllers.pid_gravity import PIDGravityCompController
from environment import Environment, EnvironmentConfig
from error_models import SensorNoiseModel
from robot.arm import Arm
from robot.link import Link
from visualization import MatplotlibVisualizer


@dataclass
class ArmBuildConfig:
    lengths: Sequence[float]
    masses: Sequence[float]
    start_angles: Sequence[float]
    max_torques: Sequence[float]
    base_pivot: tuple[float, float] = (0.0, 0.0)
    thickness: float = 0.06


@dataclass
class SimulationConfig:
    physics_dt: float = 1.0 / 120.0
    render_fps: int = 40
    max_substeps_per_frame: int = 8
    max_wall_step: float = 0.25
    history_window_s: float = 12.0
    duration_s: float | None = None


class JointTrajectory:
    """Base trajectory interface used by the simulation."""

    def sample(self, t: float, joint_count: int) -> JointTarget:
        raise NotImplementedError


class StaticJointTrajectory(JointTrajectory):
    """Constant joint-angle target (setpoint regulation)."""

    def __init__(self, target_angles: Sequence[float]) -> None:
        if not target_angles:
            raise ValueError("target_angles cannot be empty.")
        self.target_angles = [float(value) for value in target_angles]

    def sample(self, t: float, joint_count: int) -> JointTarget:
        del t
        if joint_count != len(self.target_angles):
            raise ValueError("Trajectory dimension mismatch.")
        zeros = [0.0 for _ in range(joint_count)]
        return JointTarget(
            angles=list(self.target_angles),
            velocities=zeros,
            accelerations=zeros,
        )


class MinimumJerkPointToPointTrajectory(JointTrajectory):
    """
    Smooth non-oscillatory point-to-point reference.

    Uses a quintic polynomial time-scaling (minimum-jerk style):
    s(u) = 10u^3 - 15u^4 + 6u^5, u in [0, 1]
    """

    def __init__(
        self,
        start_angles: Sequence[float],
        goal_angles: Sequence[float],
        move_start_s: float = 0.5,
        move_duration_s: float = 2.0,
    ) -> None:
        if not start_angles or not goal_angles:
            raise ValueError("start_angles and goal_angles cannot be empty.")
        if len(start_angles) != len(goal_angles):
            raise ValueError("start_angles and goal_angles must have the same length.")
        if move_duration_s <= 0.0:
            raise ValueError("move_duration_s must be positive.")

        self.start_angles = [float(value) for value in start_angles]
        self.goal_angles = [float(value) for value in goal_angles]
        self.move_start_s = float(move_start_s)
        self.move_duration_s = float(move_duration_s)
        self._joint_count = len(self.start_angles)

    def sample(self, t: float, joint_count: int) -> JointTarget:
        if joint_count != self._joint_count:
            raise ValueError("Trajectory dimension mismatch.")

        move_end_s = self.move_start_s + self.move_duration_s
        if t <= self.move_start_s:
            zeros = [0.0 for _ in range(joint_count)]
            return JointTarget(
                angles=list(self.start_angles),
                velocities=zeros,
                accelerations=zeros,
            )
        if t >= move_end_s:
            zeros = [0.0 for _ in range(joint_count)]
            return JointTarget(
                angles=list(self.goal_angles),
                velocities=zeros,
                accelerations=zeros,
            )

        u = (t - self.move_start_s) / self.move_duration_s
        u2 = u * u
        u3 = u2 * u
        u4 = u3 * u
        u5 = u4 * u

        s = 10.0 * u3 - 15.0 * u4 + 6.0 * u5
        ds_dt = (30.0 * u2 - 60.0 * u3 + 30.0 * u4) / self.move_duration_s
        d2s_dt2 = (60.0 * u - 180.0 * u2 + 120.0 * u3) / (self.move_duration_s ** 2)

        angles: List[float] = []
        velocities: List[float] = []
        accelerations: List[float] = []
        for start_angle, goal_angle in zip(self.start_angles, self.goal_angles):
            delta = goal_angle - start_angle
            angles.append(start_angle + delta * s)
            velocities.append(delta * ds_dt)
            accelerations.append(delta * d2s_dt2)

        return JointTarget(angles=angles, velocities=velocities, accelerations=accelerations)


def build_serial_arm(config: ArmBuildConfig) -> Arm:
    """Create an N-link serial arm with pivots chained from the base."""
    lengths = [float(length) for length in config.lengths]
    masses = [float(mass) for mass in config.masses]
    start_angles = [float(angle) for angle in config.start_angles]
    max_torques = [float(torque) for torque in config.max_torques]

    if not lengths:
        raise ValueError("At least one link is required.")
    if not (len(lengths) == len(masses) == len(start_angles) == len(max_torques)):
        raise ValueError("lengths, masses, start_angles, and max_torques must have equal size.")

    absolute_start_angles: List[float] = []
    cumulative_angle = 0.0
    for angle in start_angles:
        cumulative_angle += angle
        absolute_start_angles.append(cumulative_angle)

    pivot_points: List[tuple[float, float]] = [config.base_pivot]
    absolute_angle = absolute_start_angles[0]
    for idx in range(1, len(lengths)):
        prev_pivot = pivot_points[idx - 1]
        prev_length = lengths[idx - 1]
        pivot_points.append(
            (
                prev_pivot[0] + prev_length * math.cos(absolute_angle),
                prev_pivot[1] + prev_length * math.sin(absolute_angle),
            )
        )
        absolute_angle = absolute_start_angles[idx]

    links = [
        Link(
            length=length,
            mass=mass,
            pivot_point=pivot,
            start_angle=angle,
            thickness=config.thickness,
        )
        for length, mass, pivot, angle in zip(
            lengths, masses, pivot_points, absolute_start_angles
        )
    ]
    return Arm(links=links, motors=max_torques)


def make_controller(
    controller_name: str,
    lengths: Sequence[float],
    masses: Sequence[float],
    gravity_magnitude: float,
) -> ControllerBase:
    """Factory for available controller variants."""
    joint_count = len(lengths)
    name = controller_name.lower()

    if name == "open_loop":
        profile = SinusoidalTorqueProfile(
            amplitudes=[14.0 for _ in range(joint_count)],
            frequencies_hz=[0.25 + 0.08 * idx for idx in range(joint_count)],
            phases=[0.6 * idx for idx in range(joint_count)],
            offsets=[0.0 for _ in range(joint_count)],
        )
        return OpenLoopController(profile=profile)

    if name == "pid":
        return PIDController(
            joint_count=joint_count,
            kp=[95.0 for _ in range(joint_count)],
            ki=[20.0 for _ in range(joint_count)],
            kd=[14.0 for _ in range(joint_count)],
            integral_limits=[2.0 for _ in range(joint_count)],
        )

    if name == "pid_gravity":
        return PIDGravityCompController(
            joint_count=joint_count,
            kp=[80.0 for _ in range(joint_count)],
            ki=[14.0 for _ in range(joint_count)],
            kd=[12.0 for _ in range(joint_count)],
            link_lengths=lengths,
            link_masses=masses,
            gravity_magnitude=gravity_magnitude,
            integral_limits=[2.0 for _ in range(joint_count)],
            gravity_scale=1.0,
        )

    if name == "feedforward":
        return FeedforwardController(
            joint_count=joint_count,
            inertia_terms=[4.0 for _ in range(joint_count)],
            damping_terms=[1.6 for _ in range(joint_count)],
            proportional_trim=[15.0 for _ in range(joint_count)],
            link_lengths=lengths,
            link_masses=masses,
            gravity_magnitude=gravity_magnitude,
        )

    valid = ("open_loop", "pid", "pid_gravity", "feedforward")
    raise ValueError(f"Unknown controller '{controller_name}'. Choose one of {valid}.")


class Simulation:
    """Runs fixed-step pymunk physics and animated matplotlib visualization."""

    def __init__(
        self,
        environment: Environment,
        arm: Arm,
        controller: ControllerBase,
        trajectory: JointTrajectory,
        config: SimulationConfig | None = None,
        error_model: SensorNoiseModel | None = None,
    ) -> None:
        self.environment = environment
        self.space = environment.space
        self.arm = arm
        self.controller = controller
        self.trajectory = trajectory
        self.config = config or SimulationConfig()
        self.error_model = error_model
        self.sim_time = 0.0
        self.last_applied_torques = [0.0 for _ in range(self.arm.joint_count)]

        if self.controller.joint_count != self.arm.joint_count:
            raise ValueError("controller.joint_count must match arm.joint_count.")

        self.arm.add_to_space(self.space)
        self._init_history()
        self.visualizer = self._build_visualizer()
        self._animation: FuncAnimation | None = None

    def _build_visualizer(self) -> MatplotlibVisualizer:
        total_length = sum(link.length for link in self.arm.links)
        base = self.arm.links[0].pivot_point
        margin = 0.30 * total_length + 0.3
        x_limits = (base[0] - total_length - margin, base[0] + total_length + margin)
        y_limits = (base[1] - total_length - margin, base[1] + total_length + margin)
        return MatplotlibVisualizer(
            joint_count=self.arm.joint_count,
            history_window_s=self.config.history_window_s,
            arm_xlim=x_limits,
            arm_ylim=y_limits,
        )

    def _init_history(self) -> None:
        history_length = max(25, int(self.config.history_window_s / self.config.physics_dt) + 5)
        self.time_history: Deque[float] = deque(maxlen=history_length)
        self.actual_angle_history: List[Deque[float]] = [
            deque(maxlen=history_length) for _ in range(self.arm.joint_count)
        ]
        self.target_angle_history: List[Deque[float]] = [
            deque(maxlen=history_length) for _ in range(self.arm.joint_count)
        ]

    def _apply_sensor_error(self, state: JointState) -> JointState:
        if self.error_model is None:
            return JointState(angles=list(state.angles), velocities=list(state.velocities))
        return self.error_model.apply(state)

    def _record_history(self, actual_state: JointState, target: JointTarget) -> None:
        self.time_history.append(self.sim_time)
        for idx in range(self.arm.joint_count):
            self.actual_angle_history[idx].append(actual_state.angles[idx])
            self.target_angle_history[idx].append(target.angles[idx])

    def _physics_step(self) -> None:
        true_state = self.arm.state()
        measured_state = self._apply_sensor_error(
            JointState(angles=list(true_state.angles), velocities=list(true_state.velocities))
        )
        target = self.trajectory.sample(self.sim_time, self.arm.joint_count)
        command_torques = self.controller.compute_torques(
            state=measured_state,
            target=target,
            dt=self.config.physics_dt,
            sim_time=self.sim_time,
        )
        self.last_applied_torques = self.arm.apply_motor_torques(command_torques)
        self.space.step(self.config.physics_dt)
        self.sim_time += self.config.physics_dt
        self._record_history(actual_state=self.arm.state(), target=target)

    def _render_frame(self) -> tuple:
        return self.visualizer.update(
            time_values=list(self.time_history),
            measured_angles_per_joint=[list(values) for values in self.actual_angle_history],
            target_angles_per_joint=[list(values) for values in self.target_angle_history],
            arm_points=self.arm.joint_positions(),
            controller_name=self.controller.name,
            noise_enabled=self.error_model is not None
            and (self.error_model.angle_std_rad > 0.0 or self.error_model.velocity_std_rad_s > 0.0),
        )

    def run(self) -> None:
        accumulator = 0.0
        last_time = time.perf_counter()

        def init() -> tuple:
            target = self.trajectory.sample(0.0, self.arm.joint_count)
            self._record_history(actual_state=self.arm.state(), target=target)
            return self._render_frame()

        def update(_frame: int) -> tuple:
            nonlocal accumulator, last_time

            if self.visualizer.closed:
                if self._animation is not None:
                    self._animation.event_source.stop()
                return ()

            current_time = time.perf_counter()
            elapsed = min(current_time - last_time, self.config.max_wall_step)
            last_time = current_time
            accumulator += elapsed

            substeps = 0
            while (
                accumulator >= self.config.physics_dt
                and substeps < self.config.max_substeps_per_frame
            ):
                self._physics_step()
                accumulator -= self.config.physics_dt
                substeps += 1

            if substeps == self.config.max_substeps_per_frame and accumulator >= self.config.physics_dt:
                accumulator = 0.0

            artists = self._render_frame()
            if self.config.duration_s is not None and self.sim_time >= self.config.duration_s:
                if self._animation is not None:
                    self._animation.event_source.stop()
            return artists

        self._animation = FuncAnimation(
            self.visualizer.fig,
            update,
            init_func=init,
            interval=1000.0 / float(self.config.render_fps),
            blit=False,
            cache_frame_data=False,
        )
        plt.show()
