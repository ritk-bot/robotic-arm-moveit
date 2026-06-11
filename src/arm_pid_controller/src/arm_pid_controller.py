#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class PIDController(Node):

    def __init__(self):

        super().__init__('pid_controller')
        self.declare_parameter("kp", 10.0)
        self.declare_parameter("ki", 0.5)
        self.declare_parameter("kd", 0.1)
        self.declare_parameter("target", 0.5)

        # Desired shoulder angle (radians)
        self.target = self.get_parameter("target").value

        # Controller gains
        self.kp = self.get_parameter("kp").value
        self.ki = self.get_parameter("ki").value
        self.kd = self.get_parameter("kd").value

        self.integral = 0.0


        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/effort_controller/commands',
            10
        )

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.get_logger().info("PID Controller Started")

    def joint_callback(self, msg):

        self.kp = self.get_parameter("kp").value
        self.ki = self.get_parameter("ki").value
        self.kd = self.get_parameter("kd").value

        self.target = self.get_parameter("target").value
        
        if 'shoulder_joint' not in msg.name:
            return

        idx = msg.name.index('shoulder_joint')

        position = msg.position[idx]

        current_time = self.get_clock().now()

        dt = (
            current_time - self.prev_time
        ).nanoseconds / 1e9

        if dt <= 0.0:
            return

        error = self.target - position

        derivative = (error - self.prev_error) / dt

        self.integral += error * dt

        #Integral windup guard 
        self.integral = max(min(self.integral, 10.0), -10.0)

        effort = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        # effort clamp
        effort = max(min(effort, 50.0), -50.0)

        cmd = Float64MultiArray()

        cmd.data = [
            0.0,      # base_yaw
            effort,   # shoulder
            0.0,      # elbow
            0.0,      # wrist_pitch
            0.0,      # wrist_roll
            0.0       # gripper
        ]

        self.publisher.publish(cmd)

        self.get_logger().info(
        f"pos={position:.3f} "
        f"err={error:.3f} "
        f"effort={effort:.3f}"
        )

        self.prev_error = error
        self.prev_time = current_time



def main():

    rclpy.init()

    node = PIDController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()