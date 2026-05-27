import math
from typing import Tuple

import pymunk


class Link:
    """Single rigid link modeled as a segment body in pymunk."""

    def __init__(
        self,
        length: float,
        mass: float,
        pivot_point: Tuple[float, float],
        start_angle: float,
        thickness: float = 0.05,
        friction: float = 0.7,
        elasticity: float = 0.0,
    ) -> None:
        self.length = float(length)
        self.mass = float(mass)
        self.pivot_point = (float(pivot_point[0]), float(pivot_point[1]))
        self.start_angle = float(start_angle)
        self.thickness = float(thickness)

        moment_of_inertia = pymunk.moment_for_segment(
            self.mass,
            self.local_start_anchor,
            self.local_end_anchor,
            self.thickness,
        )

        self.body = pymunk.Body(self.mass, moment_of_inertia)
        com_x = self.pivot_point[0] + (self.length / 2.0) * math.cos(self.start_angle)
        com_y = self.pivot_point[1] + (self.length / 2.0) * math.sin(self.start_angle)
        self.body.position = (com_x, com_y)
        self.body.angle = self.start_angle

        self.shape = pymunk.Segment(
            self.body,
            self.local_start_anchor,
            self.local_end_anchor,
            self.thickness,
        )
        self.shape.friction = float(friction)
        self.shape.elasticity = float(elasticity)

    @property
    def local_start_anchor(self) -> Tuple[float, float]:
        """Local coordinate of the inboard joint (left end)."""
        return (-self.length / 2.0, 0.0)

    @property
    def local_end_anchor(self) -> Tuple[float, float]:
        """Local coordinate of the outboard joint (right end)."""
        return (self.length / 2.0, 0.0)

    def world_start(self) -> pymunk.Vec2d:
        """World coordinate of the inboard joint."""
        return self.body.local_to_world(self.local_start_anchor)

    def world_end(self) -> pymunk.Vec2d:
        """World coordinate of the outboard joint."""
        return self.body.local_to_world(self.local_end_anchor)

    def add_to_space(self, space: pymunk.Space) -> None:
        """Add this link body and shape to a pymunk space."""
        space.add(self.body, self.shape)