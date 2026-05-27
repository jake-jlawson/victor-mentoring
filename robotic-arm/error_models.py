from __future__ import annotations

import random
from dataclasses import dataclass
from typing import List

from controllers.base import JointState


@dataclass
class SensorNoiseModel:
    """Gaussian sensor-noise model applied to measured joint state."""

    angle_std_rad: float = 0.0
    velocity_std_rad_s: float = 0.0
    seed: int | None = 0

    def __post_init__(self) -> None:
        if self.angle_std_rad < 0.0 or self.velocity_std_rad_s < 0.0:
            raise ValueError("Noise standard deviations must be non-negative.")
        self._rng = random.Random(self.seed)

    def apply(self, state: JointState) -> JointState:
        noisy_angles: List[float] = []
        noisy_velocities: List[float] = []

        for angle in state.angles:
            noisy_angles.append(angle + self._rng.gauss(0.0, self.angle_std_rad))
        for velocity in state.velocities:
            noisy_velocities.append(velocity + self._rng.gauss(0.0, self.velocity_std_rad_s))

        return JointState(angles=noisy_angles, velocities=noisy_velocities)
