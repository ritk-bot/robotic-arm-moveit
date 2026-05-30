#!/usr/bin/env python3

import sys
import termios
import tty

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from urdf_parser_py.urdf import URDF

class KeyboardArm(Node):

    def __init__(self):
        super().__init__("keyboard_arm")

        self.pub = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            10
        )

        self.joint_names = [
            "base_yaw_joint",
            "shoulder_joint",
            "elbow_joint",
            "wrist_pitch_joint",
            "wrist_roll_joint",
            "gripper_servo_joint"
        ]

        # Current commanded positions
        self.pos = [0.0, 0.8, -1.4, 0.6, 0.0, 0.0]

#         self.pos = [
#     0.0,    # base
#     0.4,    # shoulder
#     -0.8,   # elbow
#     0.4,    # wrist pitch
#     0.0,    # wrist roll
#     0.0     # gripper
# ]

        self.step = 0.1

        self.print_help()
        self.limits = self.load_joint_limits()

    def print_help(self):
        print("""
================ ARM TELEOP ================

Q / A : Base Yaw
W / S : Shoulder
E / D : Elbow
R / F : Wrist Pitch
T / G : Wrist Roll
Y / H : Gripper

Z : Home Position
X : Exit

Current step size = 0.1 rad

============================================
""")
    
    def publish_trajectory(self):
        msg = JointTrajectory()

        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.pos
        point.time_from_start = Duration(sec=1)

        msg.points.append(point)

        self.pub.publish(msg)
        print(
            f"\r{[round(x, 2) for x in self.pos]}",
            end="",
            flush=True
        )

    def handle_key(self, key):

        if key == 'q':
            self.pos[0] += self.step
        elif key == 'a':
            self.pos[0] -= self.step

        elif key == 'w':
            self.pos[1] += self.step
        elif key == 's':
            self.pos[1] -= self.step

        elif key == 'e':
            self.pos[2] += self.step
        elif key == 'd':
            self.pos[2] -= self.step

        elif key == 'r':
            self.pos[3] += self.step
        elif key == 'f':
            self.pos[3] -= self.step

        elif key == 't':
            self.pos[4] += self.step
        elif key == 'g':
            self.pos[4] -= self.step

        elif key == 'y':
            self.pos[5] += self.step
        elif key == 'h':
            self.pos[5] -= self.step

        elif key == 'z':
            self.pos = [0.0, 0.8, -1.4, 0.6, 0.0, 0.0]

        elif key == 'x':
            return False

        else:
            return True

        self.clamp_positions()
        self.publish_trajectory()
        return True
    
    def load_joint_limits(self):

        robot = URDF.from_xml_file(
            "/home/koro/ros2_ws/src/arm/urdf/arm.urdf"
        )

        limits = {}

        for joint in robot.joints:

            if joint.name in self.joint_names:

                if joint.limit is not None:

                    limits[joint.name] = (
                        joint.limit.lower,
                        joint.limit.upper
                    )

        print("\nLoaded Joint Limits:\n")

        for joint, limit in limits.items():

            print(
                f"{joint}: "
                f"{limit[0]:.3f} -> {limit[1]:.3f}"
            )

        print()

        return limits
    def clamp_positions(self):

        for i, joint_name in enumerate(self.joint_names):

            if joint_name not in self.limits:
                continue

            lower, upper = self.limits[joint_name]

            old = self.pos[i]

            self.pos[i] = max(
                lower,
                min(upper, self.pos[i])
            )

            if old != self.pos[i]:

                print(
                    f"\n{joint_name} "
                    f"hit limit [{lower:.2f}, {upper:.2f}]"
                )

def get_key():
    fd = sys.stdin.fileno()

    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)

    finally:
        termios.tcsetattr(
            fd,
            termios.TCSADRAIN,
            old_settings
        )

    return key


def main():
    rclpy.init()

    node = KeyboardArm()

    try:
        while rclpy.ok():

            key = get_key()

            if not node.handle_key(key):
                break

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()