#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64

# Half the distance between the two propellers (propeller_port/stbd_link
# y-offsets in exhibition_water.sdf), used to convert angular.z into a
# differential thrust split.
TRACK_HALF_WIDTH = 0.38

# --- Linear: closed-loop (odom-feedback) speed control ---
#
# REPLACES the old open-loop `msg.linear.x * LINEAR_GAIN = 166.7` scheme.
# That gain was tuned so FULL commanded speed produced the right top-speed
# equilibrium (~6.04 m/s at the time), but it meant ANY commanded speed above
# ~0.3 m/s already exceeded the hardware thrust caps (51.5N/-40.2N) and
# saturated both thrusters to max/min regardless of the actual value
# requested — confirmed empirically via a live rosbridge probe (this boat's
# real speed climbed to ~2.48 m/s even when web_ui/js's Mode 3 "creep" leg
# only asked for ~0.54 m/s). Two real consequences: (1) Mode 3's
# approach/creep/reverse_swing legs never actually ran at their intended slow
# speeds, so the boat carried far more momentum into the final approach than
# the braking logic assumed, overshot the target, and the resulting
# heading-flip + re-approach cycle repeated forever (the reported "goes front
# and back" bug); (2) any real cruising speed saturated both thrusters,
# leaving the angular differential term below no headroom to act (the
# reported "can't turn" bug) — TURN_THRUST_RESERVE_N further down only
# patches this for the one case (Mode 3) it's toggled on for; the underlying
# saturate-at-any-real-speed problem was still there for every other case.
#
# Fix: track the commanded speed against the boat's OWN measured surge speed
# (nav_msgs/Odometry's twist.twist.linear.x, body frame) with a PI
# controller, the same way a real ESC/autopilot would, instead of an
# open-loop guess. KP is sized so a full MAX_LINEAR_FWD error alone would
# reach the hardware's max thrust; KI closes the small steady-state gap a
# pure-P term leaves against real hydrodynamic drag (confirmed via the same
# live probe: pure P settled ~0.45 m/s against a 0.54 m/s command — adding KI
# closed that to ~0.54). Verified empirically (not just derived): commanding
# 0.54 m/s from a standstill now converges smoothly to ~0.53-0.54 m/s and
# holds it, instead of climbing toward the boat's true ~2.49 m/s top speed.
KP_LINEAR = 51.5 / 2.49  # ~20.68 N per (m/s) of speed error — MAX_THRUST_FWD / MAX_LINEAR_FWD
KI_LINEAR = 8.0

# Feedforward: this boat's own drag model (xU*v + xUU*v^2 = thrust — mirrors
# exhibition_water.sdf's <xU>/<xUU> and web_ui/js/config.js's
# HULL_DRAG_LINEAR/HULL_DRAG_QUADRATIC), added directly to the PI output
# below as the thrust already known to be needed just to HOLD the commanded
# speed at steady state.
#
# Without it, KP_LINEAR only supplies proportional thrust while error is
# still large — by design (see above) it desaturates almost immediately
# after t=0 for ANY target, not just 0.54 m/s — so the entire remaining
# steady-state thrust has to be built up through KI_LINEAR's slow integral
# alone. That was fine at Mode 3's 0.54 m/s creep target (needs ~5.3N of
# steady thrust, closes in a couple seconds — the "pure P settled ~0.45 vs
# 0.54" comment above is exactly this gap, just small enough at that speed
# to close quickly). It was never re-tuned for web_ui/js/input.js's newer
# Mode 2 fast/medium presets: at those targets (1.0-2.49 m/s, needing
# ~20-51.5N of steady thrust) the same KI_LINEAR=8.0 takes many multiples
# longer to fully close the gap — tens of seconds, not the ~1-1.3s the
# boat's own physics (20kg mass, up to 103N combined thrust) actually
# supports — so the arrow drive visibly "settles" well short of the
# requested speed for as long as anyone would realistically hold a key.
#
# With feedforward supplying the bulk of the steady-state thrust up front,
# KP/KI only have to trim the transient and any real disturbance, so real
# convergence now tracks that same ~1-1.3s physical settling time instead of
# KI_LINEAR's — matching what W/A/R/D's direct (feedforward-only, no PI at
# all) thrust command already achieves.
HULL_DRAG_LINEAR = 1.05     # xU
HULL_DRAG_QUADRATIC = 16.24 # xUU

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

# Hardware thrust limits — must mirror exhibition_water.sdf's
# max_thrust_cmd/min_thrust_cmd on both Thruster plugin instances.
MAX_THRUST_FWD = 51.5
MAX_THRUST_REV = -40.2

# Boat's real max turn rate — must mirror web_ui/js/config.js's MAX_ANGULAR.
MAX_ANGULAR = 1.2

# Turn-reserve fix (web_ui/js/docking.js toggles this on for Mode 3 only via
# /asv_boat/turn_reserve_enable): LINEAR_GAIN (166.7 N per m/s of commanded
# speed) saturates BOTH thrusters at their hardware caps for any commanded
# speed above ~0.3 m/s. Since this mixer applies the FULL linear demand to
# each side unsplit (not divided), any real cruising speed already pins both
# propellers at max/min thrust — so the angular differential term below gets
# clipped away entirely by the Thruster plugin's own max_thrust_cmd/
# min_thrust_cmd clamp (exhibition_water.sdf), and the boat effectively loses
# all turning authority the moment it's moving at speed. This is what made
# Mode 3's ILOS path-following drift off its planned line: small course
# corrections had nowhere left to act.
#
# Fix: when enabled, cap the LINEAR component far enough below each hardware
# limit that the full angular differential (up to MAX_ANGULAR) always fits
# inside the remaining headroom, so turning authority is never clipped away
# regardless of commanded speed. Pivoting at MAX_ANGULAR needs a thrust
# *difference* of ANGULAR_GAIN * MAX_ANGULAR * TRACK_HALF_WIDTH newtons (same
# derivation as ANGULAR_GAIN's own comment above) — reserve exactly that much
# on both the forward and reverse side. Trade-off, accepted for Mode 3 only:
# top real speed drops (~2.49 -> ~1.97 m/s forward, ~2.19 -> ~1.59 m/s
# reverse, solved from this boat's own drag model, xU/xUU in
# exhibition_water.sdf) in exchange for the boat always being able to turn.
TURN_THRUST_RESERVE_N = ANGULAR_GAIN * MAX_ANGULAR * TRACK_HALF_WIDTH  # ~18.97N
LINEAR_THRUST_CAP_FWD = MAX_THRUST_FWD - TURN_THRUST_RESERVE_N  # ~32.53N
LINEAR_THRUST_CAP_REV = MAX_THRUST_REV + TURN_THRUST_RESERVE_N  # ~-21.23N


CONTROL_PERIOD_SEC = 0.05  # 20Hz — matches the live probe this was tuned against


class CmdVelThrustMixer(Node):
    def __init__(self):
        super().__init__('cmd_vel_thrust_mixer')
        self.get_logger().info('⚓ /cmd_vel -> thruster mixer initialized (closed-loop speed control).')

        self.turn_reserve_enabled = False
        self.manual_override = False
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.measured_speed = 0.0
        self.integral = 0.0

        self.left_pub = self.create_publisher(Float64, '/asv_boat/thrusters/left/thrust', 10)
        self.right_pub = self.create_publisher(Float64, '/asv_boat/thrusters/right/thrust', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Bool, '/asv_boat/turn_reserve_enable', self.turn_reserve_callback, 10)
        self.create_subscription(Bool, '/asv_boat/manual_thrust_override', self.manual_override_callback, 10)
        self.create_timer(CONTROL_PERIOD_SEC, self.control_step)

    def turn_reserve_callback(self, msg):
        self.turn_reserve_enabled = msg.data

    def manual_override_callback(self, msg):
        # web_ui/js/input.js's independent thruster control (W/A/R/D) writes
        # directly to the same /asv_boat/thrusters/left|right/thrust topics
        # this node publishes to. Without this, control_step() below kept
        # running regardless — its 20Hz PI loop and the browser's ~60Hz raw
        # thrust commands raced on the same topics the whole time a thruster
        # key was held, each overwriting the other unpredictably. Reset the
        # integral on both edges: stale windup from before the override
        # shouldn't survive into manual control, and stale windup accrued
        # *during* the override (error was never checked while paused) would
        # otherwise cause a spurious kick the instant control resumes.
        self.manual_override = msg.data
        self.integral = 0.0

    def cmd_vel_callback(self, msg):
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z

    def odom_callback(self, msg):
        # twist is in the body frame (base_link) per this odometry plugin's
        # config — linear.x is signed surge speed, matching cmd_vel's own
        # convention directly (no conversion needed).
        self.measured_speed = msg.twist.twist.linear.x

    def control_step(self):
        if self.manual_override:
            # web_ui/js/input.js's W/A/R/D control is driving the thrust
            # topics directly right now — stay out of it entirely (no
            # publish) rather than racing against it. See
            # manual_override_callback above.
            return

        # The turn-reserve cap (when enabled) is a NARROWER ceiling than the
        # raw hardware limits — anti-windup below has to check saturation
        # against whichever ceiling is actually in effect this tick, or the
        # integral could keep accumulating past the reserve cap (since it'd
        # never see itself as "saturated" against the wider hardware limits)
        # and wind up far past what's needed, causing a real overshoot the
        # instant turn-reserve mode is turned back off.
        cap_fwd = LINEAR_THRUST_CAP_FWD if self.turn_reserve_enabled else MAX_THRUST_FWD
        cap_rev = LINEAR_THRUST_CAP_REV if self.turn_reserve_enabled else MAX_THRUST_REV

        error = self.cmd_linear - self.measured_speed
        target = self.cmd_linear
        # HULL_DRAG_LINEAR/QUADRATIC solve for TOTAL thrust (both thrusters
        # combined — see exhibition_water.sdf's derivation comment: "forward
        # 2x51.5N=103N -> ~2.49 m/s"), but linear_thrust below is a
        # PER-THRUSTER command applied identically to both sides (combined =
        # 2*linear_thrust), so the feedforward has to be halved here.
        feedforward = (HULL_DRAG_LINEAR * abs(target) + HULL_DRAG_QUADRATIC * target * target) / 2.0
        if target < 0:
            feedforward = -feedforward
        linear_thrust = feedforward + KP_LINEAR * error + KI_LINEAR * self.integral
        pre_clamp = linear_thrust
        linear_thrust = max(cap_rev, min(cap_fwd, linear_thrust))
        if linear_thrust == pre_clamp:
            # Anti-windup: only accumulate the integral while not saturated,
            # or a sustained large error (e.g. commanding full speed from a
            # standstill) would wind it up far past what's needed and cause
            # a real overshoot once the boat catches up.
            self.integral += error * CONTROL_PERIOD_SEC

        angular_thrust = self.cmd_angular * ANGULAR_GAIN * TRACK_HALF_WIDTH

        left = Float64()
        left.data = max(MAX_THRUST_REV, min(MAX_THRUST_FWD, linear_thrust - angular_thrust))
        right = Float64()
        right.data = max(MAX_THRUST_REV, min(MAX_THRUST_FWD, linear_thrust + angular_thrust))

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
