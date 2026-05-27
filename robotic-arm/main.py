from __future__ import annotations

import argparse

from environment import Environment, EnvironmentConfig
from error_models import SensorNoiseModel
from simulation import (
    ArmBuildConfig,
    MinimumJerkPointToPointTrajectory,
    Simulation,
    SimulationConfig,
    StaticJointTrajectory,
    build_serial_arm,
    make_controller,
)


def _expand(value: float, count: int) -> list[float]:
    return [float(value) for _ in range(count)]


def _decay_per_joint(value: float, count: int, ratio: float = 0.9) -> list[float]:
    return [float(value) * (ratio ** idx) for idx in range(count)]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Robotic arm simulation with controller variants.")
    parser.add_argument(
        "--controller",
        default="pid",
        choices=["open_loop", "pid", "pid_gravity", "feedforward"],
        help="Controller strategy to run.",
    )
    parser.add_argument(
        "--links",
        type=int,
        default=1,
        help="Number of links/joints in the serial arm (>=1).",
    )
    parser.add_argument(
        "--noise-angle-std",
        type=float,
        default=0.0,
        help="Sensor noise std-dev for angle measurement [rad].",
    )
    parser.add_argument(
        "--noise-velocity-std",
        type=float,
        default=0.0,
        help="Sensor noise std-dev for angular velocity measurement [rad/s].",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Optional simulation duration in seconds.",
    )
    parser.add_argument(
        "--reference-mode",
        default="static",
        choices=["static", "point_to_point"],
        help="Target generator: static setpoint or smooth point-to-point move.",
    )
    parser.add_argument(
        "--target-angle",
        type=float,
        default=0.35,
        help="Static setpoint angle [rad] used by static mode and as start for point_to_point.",
    )
    parser.add_argument(
        "--goal-angle",
        type=float,
        default=0.90,
        help="Goal angle [rad] for point_to_point mode.",
    )
    parser.add_argument(
        "--move-start",
        type=float,
        default=0.5,
        help="Time [s] when the point_to_point move starts.",
    )
    parser.add_argument(
        "--move-duration",
        type=float,
        default=2.0,
        help="Duration [s] of the point_to_point move.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.links <= 0:
        raise ValueError("--links must be >= 1.")

    lengths = [max(0.45, 1.2 - 0.10 * idx) for idx in range(args.links)]
    masses = [max(0.55, 1.6 - 0.15 * idx) for idx in range(args.links)]
    max_torques = [max(10.0, 40.0 - 4.0 * idx) for idx in range(args.links)]
    start_angles = _expand(0.15, args.links)

    arm_config = ArmBuildConfig(
        lengths=lengths,
        masses=masses,
        start_angles=start_angles,
        max_torques=max_torques,
        base_pivot=(0.0, 0.0),
        thickness=0.06,
    )
    arm = build_serial_arm(arm_config)

    environment = Environment(
        config=EnvironmentConfig(
            gravity=(0.0, -9.81),
            damping=0.995,
        )
    )

    start_targets = _decay_per_joint(args.target_angle, args.links)
    goal_targets = _decay_per_joint(args.goal_angle, args.links)
    if args.reference_mode == "static":
        trajectory = StaticJointTrajectory(target_angles=start_targets)
    else:
        trajectory = MinimumJerkPointToPointTrajectory(
            start_angles=start_targets,
            goal_angles=goal_targets,
            move_start_s=args.move_start,
            move_duration_s=args.move_duration,
        )

    controller = make_controller(
        controller_name=args.controller,
        lengths=lengths,
        masses=masses,
        gravity_magnitude=abs(environment.config.gravity[1]),
    )

    noise_model = None
    if args.noise_angle_std > 0.0 or args.noise_velocity_std > 0.0:
        noise_model = SensorNoiseModel(
            angle_std_rad=args.noise_angle_std,
            velocity_std_rad_s=args.noise_velocity_std,
            seed=42,
        )

    simulation = Simulation(
        environment=environment,
        arm=arm,
        controller=controller,
        trajectory=trajectory,
        config=SimulationConfig(
            physics_dt=1.0 / 120.0,
            render_fps=40,
            max_substeps_per_frame=8,
            history_window_s=12.0,
            duration_s=args.duration,
        ),
        error_model=noise_model,
    )
    simulation.run()


if __name__ == "__main__":
    main()