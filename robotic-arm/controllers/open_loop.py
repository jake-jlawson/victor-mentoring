from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence

from controllers.base import ControllerBase, JointState, JointTarget


@dataclass
class SinusoidalTorqueProfile:
    """Simple torque profile for open-loop experiments."""

    amplitudes: Sequence[float]
    frequencies_hz: Sequence[float]
    phases: Sequence[float]
    offsets: Sequence[float]

    def __post_init__(self) -> None:
        lengths = (
            len(self.amplitudes),
            len(self.frequencies_hz),
            len(self.phases),
            len(self.offsets),
        )
        if len(set(lengths)) != 1:
            raise ValueError("All profile parameter vectors must have the same length.")

    @property
    def joint_count(self) -> int:
        return len(self.amplitudes)

    def sample(self, t: float) -> List[float]:
        values: List[float] = []
        for amplitude, frequency, phase, offset in zip(
            self.amplitudes, self.frequencies_hz, self.phases, self.offsets
        ):
            values.append(float(offset) + float(amplitude) * math.sin(2.0 * math.pi * float(frequency) * t + float(phase)))
        return values


class OpenLoopController(ControllerBase):
    """Applies predefined torques and ignores measured state."""

    name = "open_loop"

    def __init__(self, profile: SinusoidalTorqueProfile) -> None:
        super().__init__(joint_count=profile.joint_count)
        self.profile = profile

    def compute_torques(
        self,
        state: JointState,
        target: JointTarget,
        dt: float,
        sim_time: float,
    ) -> List[float]:
        del state, target, dt
        return self.profile.sample(sim_time)
