#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.timer = self.create_timer(0.1, self.loop)
        self.p = 0.0

    def loop(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [self.p] * 4
        msg.name = ["base_to_arm1_joint", "arm1_to_arm2_joint", "arm2_to_arm3_joint", "arm3_to_end_effector_joint"]
        self.p += 0.01
        self.pub.publish(msg)

if __name__ == "__main__":
    rclpy.init()
    node = ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()
