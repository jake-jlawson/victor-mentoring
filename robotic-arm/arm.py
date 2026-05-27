"""
Legacy compatibility module.

The project now uses the production classes in:
- robot.link.Link
- robot.motor.Motor
- robot.arm.Arm
"""

from robot.arm import Arm
from robot.link import Link
from robot.motor import Motor

__all__ = ["Arm", "Link", "Motor"]
