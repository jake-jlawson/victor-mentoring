"""
Super-simple PyMunk + Matplotlib robotic arm demo.

Lesson goals covered:
1) How to create rigid links (upper arm + forearm).
2) How to connect links with joints.
3) How to apply forces.
4) How to step a physics simulation.
5) How to animate simulation results with Matplotlib.
"""

import math

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pymunk

# --- Simulation timing ---
DT = 1 / 120  # Physics timestep (seconds)
STEPS_PER_FRAME = 3  # Physics steps between animation frames
FPS = 40

# --- Arm geometry ---
BASE = (0.0, 0.0)
L1 = 1.6  # Upper arm length
L2 = 1.2  # Forearm length


def make_arm_world():
    """Create physics world, links, and joints."""
    space = pymunk.Space()
    space.gravity = (0.0, -2.0)  # Small gravity so motion stays easy to see
    space.damping = 0.99

    # Create upper arm body + shape
    mass1 = 1.0
    moment1 = pymunk.moment_for_segment(mass1, (-L1 / 2, 0), (L1 / 2, 0), 0.05)
    upper = pymunk.Body(mass1, moment1)
    upper.position = (BASE[0] + L1 / 2, BASE[1])
    shape1 = pymunk.Segment(upper, (-L1 / 2, 0), (L1 / 2, 0), 0.06)
    shape1.friction = 0.8

    # Create forearm body + shape
    mass2 = 0.8
    moment2 = pymunk.moment_for_segment(mass2, (-L2 / 2, 0), (L2 / 2, 0), 0.05)
    forearm = pymunk.Body(mass2, moment2)
    forearm.position = (BASE[0] + L1 + L2 / 2, BASE[1])
    shape2 = pymunk.Segment(forearm, (-L2 / 2, 0), (L2 / 2, 0), 0.06)
    shape2.friction = 0.8

    # Shoulder joint: static world <-> upper arm
    shoulder = pymunk.PinJoint(space.static_body, upper, BASE, (-L1 / 2, 0))

    # Elbow joint: upper arm <-> forearm
    elbow = pymunk.PinJoint(upper, forearm, (L1 / 2, 0), (-L2 / 2, 0))

    # Rotary springs keep the arm stable so the animation stays readable
    shoulder_spring = pymunk.DampedRotarySpring(space.static_body, upper, 0.0, 8.0, 0.8)
    elbow_spring = pymunk.DampedRotarySpring(upper, forearm, 0.0, 5.0, 0.6)

    space.add(
        upper,
        forearm,
        shape1,
        shape2,
        shoulder,
        elbow,
        shoulder_spring,
        elbow_spring,
    )

    return space, upper, forearm


def joint_positions(upper, forearm):
    """Return shoulder, elbow, wrist positions in world coordinates."""
    shoulder = pymunk.Vec2d(*BASE)
    elbow = upper.local_to_world((L1 / 2, 0))
    wrist = forearm.local_to_world((L2 / 2, 0))
    return shoulder, elbow, wrist


def apply_demo_force(forearm, frame):
    """
    Apply a small oscillating force at the wrist.
    This is the "actuation" for the lesson.
    """
    t = frame / FPS
    force = pymunk.Vec2d(0.0, 7.0 * math.sin(2.0 * math.pi * 0.6 * t))
    wrist = forearm.local_to_world((L2 / 2, 0))
    forearm.apply_force_at_world_point(force, wrist)
    return force


def main():
    space, upper, forearm = make_arm_world()

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_title("Simple 2D Robotic Arm (PyMunk + Matplotlib)")
    ax.set_xlim(-2.5, 3.5)
    ax.set_ylim(-2.5, 2.5)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)

    # Arm drawing (shoulder -> elbow -> wrist)
    (arm_line,) = ax.plot([], [], "o-", lw=4)
    force_text = ax.text(-2.4, 2.2, "", fontsize=10)

    def init():
        shoulder, elbow, wrist = joint_positions(upper, forearm)
        arm_line.set_data([shoulder.x, elbow.x, wrist.x], [shoulder.y, elbow.y, wrist.y])
        force_text.set_text("Applied wrist force: (0.00, 0.00)")
        return arm_line, force_text

    def update(frame):
        # Step physics multiple times each frame for smoother motion
        for _ in range(STEPS_PER_FRAME):
            current_force = apply_demo_force(forearm, frame)
            space.step(DT)

        shoulder, elbow, wrist = joint_positions(upper, forearm)
        xs = [shoulder.x, elbow.x, wrist.x]
        ys = [shoulder.y, elbow.y, wrist.y]
        arm_line.set_data(xs, ys)
        force_text.set_text(f"Applied wrist force: ({current_force.x:.2f}, {current_force.y:.2f})")
        return arm_line, force_text

    # Keep a reference to animation object so it is not garbage-collected.
    anim = FuncAnimation(
        fig,
        update,
        init_func=init,
        interval=1000 / FPS,
        blit=False,
        cache_frame_data=False,
    )

    plt.show()


if __name__ == "__main__":
    main()
