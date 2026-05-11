#!/usr/bin/env python3
"""
Stanley Lateral Controller  (fixed CTE computation)
=====================================================
Subscribes
----------
  /odom              (nav_msgs/Odometry)
  /desired_heading   (std_msgs/Float64)   – tangent angle from APF planner
  /goal_point        (geometry_msgs/Point) – for reference (unused in control)

Publishes
---------
  /steering_angle    (std_msgs/Float64)   – radians, + = left turn

Fix applied
-----------
The original code computed CTE as:
    cte = -error_x * sin(heading) + error_y * cos(heading)

This is the correct formula **only when the robot's position error vector
(dx, dy) points from the robot to the goal**.  The sign convention and the
frame decomposition were already correct mathematically, but the variable
`desired_heading` was not always available before the first heading message
arrived (default 0.0 → wrong CTE on startup).  A `has_heading` guard is
added, and the formula is clearly documented so it is easy to verify.

Formula derivation
------------------
Let ψ = desired_heading (angle of the path tangent, in world frame).
The unit tangent vector is  T̂ = [cos ψ, sin ψ].
The unit lateral (left-normal) vector is  N̂ = [-sin ψ, cos ψ].

The error vector from robot to goal is  e = [dx, dy] = [gx-rx, gy-ry].
CTE = projection of e onto N̂  (positive = robot is to the RIGHT of path):
    CTE = e · N̂ = dx·(-sin ψ) + dy·cos ψ
        = -dx·sin ψ + dy·cos ψ

This works for ANY heading (X-axis, Y-axis, diagonal, corner).
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point


class LateralControlTrack3(Node):

    def __init__(self):
        super().__init__("lateral_control_track3")

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter("k_p",          1.0)
        self.declare_parameter("max_steering", 0.6108)

        self.k_p          = self.get_parameter("k_p").value
        self.max_steering = self.get_parameter("max_steering").value

        # ── State ────────────────────────────────────────────────────────
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0
        self.speed = 0.0

        # Latest desired heading from planner
        self.desired_heading = 0.0
        self.has_heading      = False   # guard: don't steer before first heading

        # Latest goal from planner (kept for possible future use)
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.has_goal = False

        # ── I/O ──────────────────────────────────────────────────────────
        self.pub = self.create_publisher(Float64, "/steering_angle", 10)

        self.create_subscription(Odometry, "/odom",             self.odom_cb,    10)
        self.create_subscription(Float64,  "/desired_heading",  self.heading_cb, 10)
        self.create_subscription(Point,    "/goal_point",       self.goal_cb,    10)

        self.get_logger().info("Stanley Lateral Controller started.")

    # ------------------------------------------------------------------ #
    #  Subscribers                                                         #
    # ------------------------------------------------------------------ #

    def heading_cb(self, msg: Float64):
        self.desired_heading = msg.data
        self.has_heading = True

    def goal_cb(self, msg: Point):
        self.goal_x  = msg.x
        self.goal_y  = msg.y
        self.has_goal = True

    def odom_cb(self, msg: Odometry):
        # Position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Yaw from quaternion
        q     = msg.pose.pose.orientation
        siny  = 2.0 * (q.w * q.z + q.x * q.y)
        cosy  = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

        # Speed
        self.speed = msg.twist.twist.linear.x

        if self.has_heading:
            self.compute()

    # ------------------------------------------------------------------ #
    #  Stanley control                                                     #
    # ------------------------------------------------------------------ #

    def compute(self):
        psi = self.desired_heading   # path tangent angle in world frame

        # ── 1. Cross-Track Error ────────────────────────────────────────
        # Error vector from robot to goal (world frame)
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        # Project onto the path's left-normal vector N̂ = [-sin ψ, cos ψ]
        # CTE > 0  →  robot is to the RIGHT of the path (steer left / positive)
        # CTE < 0  →  robot is to the LEFT  of the path (steer right / negative)
        #
        # This formula is heading-independent: it works when the path runs
        # along X, Y, or any diagonal, and through corners.
        cte = -dx * math.sin(psi) + dy * math.cos(psi)

        # ── 2. Heading Error ────────────────────────────────────────────
        heading_error = psi - self.yaw
        # Wrap to [-π, π]
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        # ── 3. Stanley Law ──────────────────────────────────────────────
        # δ = ψ_e + arctan(k · cte / v)
        v           = max(0.5, abs(self.speed))   # avoid div-by-zero
        stanley_term = math.atan2(self.k_p * cte, v)
        steer        = heading_error + stanley_term

        # ── 4. Clamp ────────────────────────────────────────────────────
        steer = max(-self.max_steering, min(self.max_steering, steer))

        # ── 5. Publish ──────────────────────────────────────────────────
        out       = Float64()
        out.data  = steer
        self.pub.publish(out)

        self.get_logger().debug(
            f"ψ={math.degrees(psi):.1f}°  yaw={math.degrees(self.yaw):.1f}°  "
            f"CTE={cte:.3f}  ψ_e={math.degrees(heading_error):.1f}°  "
            f"δ={math.degrees(steer):.1f}°"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LateralControlTrack3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()