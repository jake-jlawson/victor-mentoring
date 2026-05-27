from __future__ import annotations

from typing import List, Sequence

from controllers.base import ControllerBase, JointState, JointTarget, gravity_compensation_torques


class FeedforwardController(ControllerBase):
    """
    Model-based feedforward controller.

    tau = I * q_ddot_des + B * q_dot_des + gravity(q_des)
    """

    name = "feedforward"

    def __init__(
        self,
        joint_count: int,
        inertia_terms: Sequence[float] | float,
        damping_terms: Sequence[float] | float,
        link_lengths: Sequence[float],
        link_masses: Sequence[float],
        gravity_magnitude: float,
        proportional_trim: Sequence[float] | float = 0.0,
    ) -> None:
        super().__init__(joint_count=joint_count)
        self.inertia_terms = self._expand(inertia_terms)
        self.damping_terms = self._expand(damping_terms)
        self.proportional_trim = self._expand(proportional_trim)
        self.link_lengths = [float(length) for length in link_lengths]
        self.link_masses = [float(mass) for mass in link_masses]
        self.gravity_magnitude = float(abs(gravity_magnitude))

        if len(self.link_lengths) != joint_count or len(self.link_masses) != joint_count:
            raise ValueError("link_lengths and link_masses must match joint_count.")

    def _expand(self, values: Sequence[float] | float) -> List[float]:
        if isinstance(values, (int, float)):
            return [float(values) for _ in range(self.joint_count)]
        expanded = [float(value) for value in values]
        if len(expanded) != self.joint_count:
            raise ValueError(f"Expected {self.joint_count} values, got {len(expanded)}.")
        return expanded

    def compute_torques(
        self,
        state: JointState,
        target: JointTarget,
        dt: float,
        sim_time: float,
    ) -> List[float]:
        del dt, sim_time
        self._validate_dimensions(state, target)
        gravity_terms = gravity_compensation_torques(
            joint_angles=target.angles,
            link_lengths=self.link_lengths,
            link_masses=self.link_masses,
            gravity_magnitude=self.gravity_magnitude,
        )

        torques: List[float] = []
        for i in range(self.joint_count):
            inertia_term = self.inertia_terms[i] * target.accelerations[i]
            damping_term = self.damping_terms[i] * target.velocities[i]
            trim_term = self.proportional_trim[i] * (target.angles[i] - state.angles[i])
            torques.append(inertia_term + damping_term + gravity_terms[i] + trim_term)
        return torques
