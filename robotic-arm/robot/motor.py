from __future__ import annotations

from typing import Optional

import pymunk


class Motor:
    """Joint motor model with realistic symmetric torque saturation."""

    def __init__(
        self,
        max_torque: float,
        body_a: Optional[pymunk.Body],
        body_b: pymunk.Body,
        verbose: bool = False,
    ) -> None:
        self.max_torque = float(max_torque)
        self.body_a = body_a
        self.body_b = body_b
        self.torque = 0.0
        self.verbose = verbose

    def apply_torque(self, requested_torque: float) -> float:
        """Apply saturation and return the torque that will be used."""
        torque = float(requested_torque)
        saturated = max(-self.max_torque, min(self.max_torque, torque))
        self.torque = saturated

        if self.verbose and saturated != torque:
            print(f"Requested torque {torque:.3f} saturated to {saturated:.3f}")
        elif self.verbose:
            print(f"Applying torque: {saturated:.3f}")

        return self.torque