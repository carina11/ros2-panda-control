#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class ZAxisMover(Node):
    def __init__(self):
        super().__init__('z_axis_mover')
        self.pub = self.create_publisher(TwistStamped, '/delta_twist_cmds', 10)

        self.frame_id = 'panda_link0'  # Match planning_frame
        self.speed = 0.05  # m/s upward in Z

        self.duration_sec = 3.0
        self.duration_ns = int(self.duration_sec * 1e9)

        self.start_time_ns = self.get_clock().now().nanoseconds

        self.done = False
        self.timer = self.create_timer(0.05, self.send_twist)

        self.get_logger().info(f"Moving robot up in Z at {self.speed} m/s for {self.duration_sec} seconds.")

    def send_twist(self):
        if self.done:
            return

        now_ns = self.get_clock().now().nanoseconds
        elapsed_ns = now_ns - self.start_time_ns

        if elapsed_ns > self.duration_ns:
            self.send_zero_twist()
            self.get_logger().info("Movement complete.")
            self.done = True
            return

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.z = self.speed
        self.pub.publish(msg)

    def send_zero_twist(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ZAxisMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# ✅ This is the missing piece
if __name__ == "__main__":
    main()
