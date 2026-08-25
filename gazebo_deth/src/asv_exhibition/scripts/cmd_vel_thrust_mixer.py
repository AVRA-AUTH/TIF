#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# Half the distance between the two propellers (propeller_port/stbd_link
# y-offsets in exhibition_water.sdf), used to convert angular.z into a
# differential thrust split.
TRACK_HALF_WIDTH = 0.38

# Newtons of thrust per (m/s or rad/s) of commanded velocity — tuned
# alongside the Thruster/SimpleHydrodynamics coefficients in
# exhibition_water.sdf (Phase 5), not derived independently here.
LINEAR_GAIN = 166.7
ANGULAR_GAIN = 166.7


class CmdVelThrustMixer(Node):
    def __init__(self):
        super().__init__('cmd_vel_thrust_mixer')
        self.get_logger().info('⚓ /cmd_vel -> thruster mixer initialized.')

        self.left_pub = self.create_publisher(Float64, '/asv_boat/thrusters/left/thrust', 10)
        self.right_pub = self.create_publisher(Float64, '/asv_boat/thrusters/right/thrust', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        linear_thrust = msg.linear.x * LINEAR_GAIN
        angular_thrust = msg.angular.z * ANGULAR_GAIN * TRACK_HALF_WIDTH

        left = Float64()
        left.data = linear_thrust - angular_thrust
        right = Float64()
        right.data = linear_thrust + angular_thrust

        self.left_pub.publish(left)
        self.right_pub.publish(right)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelThrustMixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
