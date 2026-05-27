from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import pymunk


@dataclass
class EnvironmentConfig:
    gravity: Tuple[float, float] = (0.0, -140.0)
    damping: float = 0.995


class Environment:
    """Owns and configures the pymunk simulation space."""

    def __init__(self, config: EnvironmentConfig | None = None) -> None:
        self.config = config or EnvironmentConfig()
        self.space = self._create_space()

    def _create_space(self) -> pymunk.Space:
        space = pymunk.Space()
        space.gravity = self.config.gravity
        space.damping = self.config.damping
        return space
