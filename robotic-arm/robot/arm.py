from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

import pymunk

from robot.link import Link
from robot.motor import Motor


@dataclass
class ArmState:
    """Controller-facing arm state."""

    angles: List[float]
    velocities: List[float]


class Arm:
    """
    N-link planar robotic arm assembled from `Link` and `Motor` objects.
    """

    def __init__(self, links: Sequence[Link], motors: Sequence[Motor | float]) -> None:
        if not links:
            raise ValueError("Arm requires at least one link.")

        self.links = list(links)
        self.joints: List[pymunk.Constraint] = []
        self.motors = self._build_motors(motors)
        self._space: pymunk.Space | None = None

    @property
    def joint_count(self) -> int:
        return len(self.links)

    def _build_motors(self, motors: Sequence[Motor | float]) -> List[Motor]:
        if len(motors) != len(self.links):
            raise ValueError("Expected one motor per joint (same count as links).")

        built_motors: List[Motor] = []
        for idx, motor in enumerate(motors):
            parent_body = None if idx == 0 else self.links[idx - 1].body
            child_body = self.links[idx].body

            if isinstance(motor, Motor):
                if motor.body_a is None:
                    motor.body_a = parent_body
                if motor.body_b is None:
                    motor.body_b = child_body
                built_motors.append(motor)
                continue

            built_motors.append(
                Motor(
                    max_torque=float(motor),
                    body_a=parent_body,
                    body_b=child_body,
                )
            )

        return built_motors

    def add_to_space(self, space: pymunk.Space) -> None:
        """Add links and revolute joints to the pymunk world."""
        if self._space is not None and self._space is not space:
            raise RuntimeError("Arm is already attached to another space.")

        for link in self.links:
            link.add_to_space(space)

        self.joints.clear()

        base_link = self.links[0]
        base_joint = pymunk.PivotJoint(
            space.static_body,
            base_link.body,
            base_link.pivot_point,
            base_link.local_start_anchor,
        )
        self.joints.append(base_joint)

        for idx in range(1, len(self.links)):
            parent = self.links[idx - 1]
            child = self.links[idx]
            joint = pymunk.PivotJoint(
                parent.body,
                child.body,
                parent.local_end_anchor,
                child.local_start_anchor,
            )
            self.joints.append(joint)

        space.add(*self.joints)
        self._space = space

    def apply_motor_torques(self, torques: Sequence[float]) -> List[float]:
        """Apply one torque command per joint and return saturated torques."""
        if len(torques) != len(self.motors):
            raise ValueError("Expected one torque value per motor.")

        applied: List[float] = []
        for motor, requested_torque in zip(self.motors, torques):
            saturated_torque = motor.apply_torque(requested_torque)

            if motor.body_a is not None:
                motor.body_a.torque -= saturated_torque
            motor.body_b.torque += saturated_torque
            applied.append(saturated_torque)

        return applied

    def joint_angles(self) -> List[float]:
        """Joint angles relative to the parent link/static base."""
        angles: List[float] = []
        for idx, link in enumerate(self.links):
            if idx == 0:
                angles.append(link.body.angle)
            else:
                parent = self.links[idx - 1]
                angles.append(link.body.angle - parent.body.angle)
        return angles

    def joint_velocities(self) -> List[float]:
        """Joint angular velocities relative to parent/static base."""
        velocities: List[float] = []
        for idx, link in enumerate(self.links):
            if idx == 0:
                velocities.append(link.body.angular_velocity)
            else:
                parent = self.links[idx - 1]
                velocities.append(link.body.angular_velocity - parent.body.angular_velocity)
        return velocities

    def state(self) -> ArmState:
        return ArmState(angles=self.joint_angles(), velocities=self.joint_velocities())

    def absolute_link_angles(self) -> List[float]:
        """Absolute orientation of each link in world frame."""
        return [link.body.angle for link in self.links]

    def joint_positions(self) -> List[Tuple[float, float]]:
        """
        Return world coordinates for all kinematic points.
        Output is [base_joint, joint_1, ..., end_effector].
        """
        points: List[Tuple[float, float]] = []
        base = self.links[0].world_start()
        points.append((base.x, base.y))
        for link in self.links:
            end = link.world_end()
            points.append((end.x, end.y))
        return points

    def set_joint_angles(self, joint_angles: Iterable[float]) -> None:
        """Set initial joint angles using relative-joint convention."""
        joint_angles_list = list(joint_angles)
        if len(joint_angles_list) != self.joint_count:
            raise ValueError("Expected one joint angle per link.")

        absolute_angle = 0.0
        for idx, (link, joint_angle) in enumerate(zip(self.links, joint_angles_list)):
            absolute_angle += float(joint_angle)
            link.body.angle = absolute_angle
            link.body.angular_velocity = 0.0

            if idx == 0:
                start = pymunk.Vec2d(*link.pivot_point)
            else:
                parent = self.links[idx - 1]
                start = parent.world_end()
            center = start + pymunk.Vec2d(link.length / 2.0, 0.0).rotated(absolute_angle)
            link.body.position = center