from __future__ import annotations

from typing import List, Sequence

from controllers.base import ControllerBase, JointState, JointTarget


def _expand_parameter(values: Sequence[float] | float, joint_count: int) -> List[float]:
    if isinstance(values, (int, float)):
        return [float(values) for _ in range(joint_count)]
    expanded = [float(value) for value in values]
    if len(expanded) != joint_count:
        raise ValueError(f"Expected {joint_count} gain values, got {len(expanded)}.")
    return expanded


class PIDController(ControllerBase):
    """Per-joint PID controller with integral clamping anti-windup."""

    name = "pid"

    def __init__(
        self,
        joint_count: int,
        kp: Sequence[float] | float,
        ki: Sequence[float] | float,
        kd: Sequence[float] | float,
        integral_limits: Sequence[float] | float = 5.0,
    ) -> None:
        super().__init__(joint_count=joint_count)
        self.kp = _expand_parameter(kp, joint_count)
        self.ki = _expand_parameter(ki, joint_count)
        self.kd = _expand_parameter(kd, joint_count)
        self.integral_limits = _expand_parameter(integral_limits, joint_count)
        self.integral_error = [0.0 for _ in range(joint_count)]

    def reset(self) -> None:
        self.integral_error = [0.0 for _ in range(self.joint_count)]

    def compute_torques(
        self,
        state: JointState,
        target: JointTarget,
        dt: float,
        sim_time: float,
    ) -> List[float]:
        del sim_time
        self._validate_dimensions(state, target)
        if dt <= 0.0:
            raise ValueError("dt must be positive.")

        torques: List[float] = []
        for i in range(self.joint_count):
            position_error = target.angles[i] - state.angles[i]
            velocity_error = target.velocities[i] - state.velocities[i]

            self.integral_error[i] += position_error * dt
            integral_limit = self.integral_limits[i]
            self.integral_error[i] = max(
                -integral_limit,
                min(integral_limit, self.integral_error[i]),
            )

            torque = (
                self.kp[i] * position_error
                + self.ki[i] * self.integral_error[i]
                + self.kd[i] * velocity_error
            )
            torques.append(torque)

        return torques
