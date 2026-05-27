from __future__ import annotations

from typing import List, Sequence

from controllers.base import JointState, JointTarget, gravity_compensation_torques
from controllers.pid import PIDController


class PIDGravityCompController(PIDController):
    """PID plus model-based gravity compensation feedforward."""

    name = "pid_gravity"

    def __init__(
        self,
        joint_count: int,
        kp: Sequence[float] | float,
        ki: Sequence[float] | float,
        kd: Sequence[float] | float,
        link_lengths: Sequence[float],
        link_masses: Sequence[float],
        gravity_magnitude: float,
        integral_limits: Sequence[float] | float = 5.0,
        gravity_scale: float = 1.0,
    ) -> None:
        super().__init__(
            joint_count=joint_count,
            kp=kp,
            ki=ki,
            kd=kd,
            integral_limits=integral_limits,
        )
        self.link_lengths = [float(length) for length in link_lengths]
        self.link_masses = [float(mass) for mass in link_masses]
        self.gravity_magnitude = float(abs(gravity_magnitude))
        self.gravity_scale = float(gravity_scale)

        if len(self.link_lengths) != joint_count or len(self.link_masses) != joint_count:
            raise ValueError("link_lengths and link_masses must match joint_count.")

    def compute_torques(
        self,
        state: JointState,
        target: JointTarget,
        dt: float,
        sim_time: float,
    ) -> List[float]:
        pid_torques = super().compute_torques(state=state, target=target, dt=dt, sim_time=sim_time)
        gravity_terms = gravity_compensation_torques(
            joint_angles=state.angles,
            link_lengths=self.link_lengths,
            link_masses=self.link_masses,
            gravity_magnitude=self.gravity_magnitude,
        )
        return [pid + self.gravity_scale * ff for pid, ff in zip(pid_torques, gravity_terms)]
