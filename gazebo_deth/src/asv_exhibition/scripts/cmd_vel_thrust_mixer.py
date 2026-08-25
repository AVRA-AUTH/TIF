#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# Half the distance between the two propellers (propeller_port/stbd_link
# y-offsets in exhibition_water.sdf), used to convert angular.z into a
# differential thrust split.
TRACK_HALF_WIDTH = 0.38

# Newtons of thrust per m/s of commanded linear velocity — tuned alongside
# the Thruster/SimpleHydrodynamics coefficients in exhibition_water.sdf
# (Phase 5) so real thrust saturation settles forward speed near
# web_ui/app.js's MAX_LINEAR_FWD target; verified empirically (~6.04 m/s).
LINEAR_GAIN = 166.7

# Differential-thrust gain: Newtons of thrust *difference* between the two
# propellers per rad/s of commanded yaw rate. Derived from
# exhibition_water.sdf's own yaw damping rather than reusing LINEAR_GAIN's
# value (the previous ANGULAR_GAIN = 166.7 was never actually derived — see
# git history): pivoting in place at MAX_ANGULAR = 1.2 rad/s needs
# steady-state torque tau = nR * MAX_ANGULAR = 12.0 * 1.2 = 14.4 N*m (nRR is
# 0 there, so no quadratic term). Each thruster sits TRACK_HALF_WIDTH from
# the centerline, so a thrust difference dF produces
# tau = 2 * TRACK_HALF_WIDTH * dF, giving dF = 14.4 / (2*0.38) ≈ 18.95N.
# ANGULAR_GAIN is then dF per rad/s of yaw *and* per meter of track width
# (mixer multiplies by TRACK_HALF_WIDTH again below), so
# ANGULAR_GAIN = dF / (MAX_ANGULAR * TRACK_HALF_WIDTH) ≈ 41.6. The old
# 166.7 produced ~4x the intended torque — spinning the boat at ~4.8 rad/s
# from a standstill instead of the requested 1.2.
ANGULAR_GAIN = 41.6


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
