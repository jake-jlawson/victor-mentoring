from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
import math
from typing import List, Sequence


@dataclass
class JointState:
    """Measured joint state used by controllers."""

    angles: List[float]
    velocities: List[float]


@dataclass
class JointTarget:
    """Desired joint references used by controllers."""

    angles: List[float]
    velocities: List[float]
    accelerations: List[float]


class ControllerBase(ABC):
    """Abstract interface for all controller variants."""

    name = "base"

    def __init__(self, joint_count: int) -> None:
        if joint_count <= 0:
            raise ValueError("joint_count must be positive.")
        self.joint_count = int(joint_count)

    def reset(self) -> None:
        """Reset any internal controller memory."""
        return None

    @abstractmethod
    def compute_torques(
        self,
        state: JointState,
        target: JointTarget,
        dt: float,
        sim_time: float,
    ) -> List[float]:
        """Return one torque command per joint."""

    def _validate_dimensions(self, state: JointState, target: JointTarget) -> None:
        expected = self.joint_count
        lengths = (
            len(state.angles),
            len(state.velocities),
            len(target.angles),
            len(target.velocities),
            len(target.accelerations),
        )
        if any(size != expected for size in lengths):
            raise ValueError(f"Controller expected vectors of length {expected}, got {lengths}.")


def gravity_compensation_torques(
    joint_angles: Sequence[float],
    link_lengths: Sequence[float],
    link_masses: Sequence[float],
    gravity_magnitude: float,
) -> List[float]:
    """
    Approximate planar gravity torques for a serial chain.

    Angles are relative joint angles (q), measured from +x axis with CCW positive.
    Gravity magnitude is positive (e.g. abs(space.gravity.y)).
    """
    joint_count = len(joint_angles)
    if len(link_lengths) != joint_count or len(link_masses) != joint_count:
        raise ValueError("link_lengths and link_masses must match joint angle count.")

    absolute_angles: List[float] = []
    theta_sum = 0.0
    for q in joint_angles:
        theta_sum += q
        absolute_angles.append(theta_sum)

    torques = [0.0 for _ in range(joint_count)]
    for i in range(joint_count):
        tau = 0.0
        for k in range(i, joint_count):
            arm_length = 0.0
            for segment in range(i, k):
                arm_length += link_lengths[segment]
            arm_length += 0.5 * link_lengths[k]
            tau += link_masses[k] * gravity_magnitude * arm_length * math.cos(absolute_angles[k])
        torques[i] = tau
    return torques
