#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class ASVNavigator(Node):
    def __init__(self):
        super().__init__('asv_navigator')
        self.get_logger().info('⚓ ASV Telemetry & Safety Monitor initialized.')

        # State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        self.min_obstacle_dist = 15.0

        # ROS 2 Subscriptions
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        if not msg.ranges:
            return
        
        # Filter out self-reflections (< 1.8m)
        valid = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r) and r > 1.8]
        if valid:
            self.min_obstacle_dist = min(valid)
            if self.min_obstacle_dist < 3.0:
                self.get_logger().warn(f"⚠️ Proximity Alert: Nearby object detected at {self.min_obstacle_dist:.2f}m")

def main(args=None):
    rclpy.init(args=args)
    node = ASVNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
