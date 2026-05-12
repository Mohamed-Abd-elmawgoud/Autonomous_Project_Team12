
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point


# ─────────────────────────────────────────────────────────────────────────────
# Track centreline waypoints (counter-clockwise)
# Format: (x, y, segment_type, straight_speed, corner_speed)
# ─────────────────────────────────────────────────────────────────────────────

# Centreline is midway between outer (0,5 / ±2) and inner (0.75,4.25 / ±1.25)
# → x ∈ [0.375, 4.625],  y ∈ [-1.625, 1.625]

CX_LEFT   = 0.375
CX_RIGHT  = 4.625
CY_TOP    = 1.625
CY_BOTTOM = -1.625

WAYPOINTS = [
    # (x,        y,          type,      v_straight, v_corner)
    (CX_LEFT,   CY_TOP,     'corner',   0.0,  0.4),   # 0  Top-Left
    (CX_RIGHT,  CY_TOP,     'corner',   0.0,  0.4),   # 1  Top-Right
    (CX_RIGHT,  CY_BOTTOM,  'corner',   0.0,  0.4),   # 2  Bot-Right
    (CX_LEFT,   CY_BOTTOM,  'corner',   0.0,  0.4),   # 3  Bot-Left
]

# Mid-waypoints along straights (used only for heading reference)
# The speed controller handles the actual driving on straights.
STRAIGHT_TARGETS = {
    # Going from WP i to WP (i+1), the planner sets a mid-point goal
    # so the heading stays sensible.
    0: (CX_RIGHT,  CY_TOP),     # top straight   → heading = 0  (east)
    1: (CX_RIGHT,  CY_BOTTOM),  # right straight  → heading = -π/2
    2: (CX_LEFT,   CY_BOTTOM),  # bottom straight → heading = π  (west)
    3: (CX_LEFT,   CY_TOP),     # left straight   → heading = π/2
}

STRAIGHT_HEADINGS = {
    0:  0.0,          # east
    1: -math.pi / 2,  # south
    2:  math.pi,      # west
    3:  math.pi / 2,  # north
}

CORNER_HEADINGS = {
    # heading to publish while *approaching* the corner waypoint
    0:  0.0,          # approaching TL: came from BL heading east (doesn't matter much, corner override)
    1: -math.pi / 2,  # approaching TR: came from TL heading south
    2:  math.pi,      # approaching BR: came from TR heading west
    3:  math.pi / 2,  # approaching BL: came from BR heading north
}

# Speed settings
STRAIGHT_SPEED   = 1.2   # m/s  – published on /desired_velocity (speed ctrl)
CORNER_SPEED     = 0.4   # m/s  – used by P2P controller internally

# Distance threshold to consider a waypoint "reached"
CORNER_REACH_DIST   = 0.30   # m
STRAIGHT_REACH_DIST = 0.35   # m

# Which segment type to use for longitudinal control
SEGMENT_SPEED    = 'straight'   # use speed controller
SEGMENT_WAYPOINT = 'corner'     # use P2P waypoint controller


class APFPlannerTrack(Node):

    def __init__(self):
        super().__init__('apf_planner_track')

        # ── Publishers ───────────────────────────────────────────────────
        self.pub_goal    = self.create_publisher(Point,   '/goal_point',       10)
        self.pub_vel     = self.create_publisher(Float64, '/desired_velocity',  10)
        self.pub_heading = self.create_publisher(Float64, '/desired_heading',   10)

        # ── Subscriber ───────────────────────────────────────────────────
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # ── State ────────────────────────────────────────────────────────
        self.x     = 0.0
        self.y     = 0.0
        self.speed = 0.0

        # Start targeting the first corner waypoint
        self.wp_idx      = 0   # index into WAYPOINTS[]
        self.on_straight = False  # True while driving a straight segment

        # ── Timer (10 Hz) ────────────────────────────────────────────────
        self.timer = self.create_timer(0.1, self.plan_cb)
        self.get_logger().info('APF Planner started — driving counter-clockwise.')

    # ── Odometry ─────────────────────────────────────────────────────────────

    def odom_cb(self, msg: Odometry):
        self.x     = msg.pose.pose.position.x
        self.y     = msg.pose.pose.position.y
        self.speed = msg.twist.twist.linear.x

    # ── Main planning loop ───────────────────────────────────────────────────

    def plan_cb(self):
        wx, wy, wtype, _, _ = WAYPOINTS[self.wp_idx]
        dist = math.hypot(wx - self.x, wy - self.y)

        # ── Determine active segment ──────────────────────────────────────
        if not self.on_straight:
            # Currently targeting a CORNER waypoint
            self._publish_corner_mode(wx, wy, dist)
        else:
            # Currently on a STRAIGHT — use speed controller
            self._publish_straight_mode(dist)

    # ── Corner mode ──────────────────────────────────────────────────────────

    def _publish_corner_mode(self, wx, wy, dist):
        """
        Send goal_point to the P2P waypoint controller.
        Publish a slow desired_velocity so the speed controller doesn't race.
        Publish the heading towards the corner goal.
        """
        # Heading: angle from robot to corner waypoint
        heading = math.atan2(wy - self.y, wx - self.x)

        self._pub_goal(wx, wy)
        self._pub_heading(heading)
        self._pub_velocity(CORNER_SPEED)

        self.get_logger().debug(
            f'[CORNER→WP{self.wp_idx}] dist={dist:.2f}  heading={math.degrees(heading):.1f}°'
        )

        # Advance when close enough
        if dist < CORNER_REACH_DIST:
            self.get_logger().info(f'Corner WP{self.wp_idx} reached — switching to straight.')
            # Transition: now drive the STRAIGHT after this corner
            self.on_straight = True

    # ── Straight mode ────────────────────────────────────────────────────────

    def _publish_straight_mode(self, dist_to_next_corner):
        """
        Use the speed controller for longitudinal.
        Publish the straight's ideal heading.
        Goal point is still the next corner (keeps lateral controller informed).
        """
        # Next corner index (wraps)
        next_idx = (self.wp_idx + 1) % len(WAYPOINTS)
        nx, ny, _, _, _ = WAYPOINTS[next_idx]

        # Straight heading
        heading = STRAIGHT_HEADINGS[self.wp_idx]

        self._pub_goal(nx, ny)
        self._pub_heading(heading)
        self._pub_velocity(STRAIGHT_SPEED)

        # Compute distance to the NEXT corner
        dist_next = math.hypot(nx - self.x, ny - self.y)

        self.get_logger().debug(
            f'[STRAIGHT→WP{next_idx}] dist={dist_next:.2f}  v={STRAIGHT_SPEED}'
        )

        # When close to the next corner, hand off to corner mode
        if dist_next < CORNER_REACH_DIST + 0.20:   # slightly earlier trigger
            self.get_logger().info(
                f'Approaching corner WP{next_idx} — switching to corner mode.'
            )
            self.wp_idx      = next_idx
            self.on_straight = False

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _pub_goal(self, x, y):
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.0
        self.pub_goal.publish(msg)

    def _pub_heading(self, angle):
        msg = Float64()
        msg.data = float(angle)
        self.pub_heading.publish(msg)

    def _pub_velocity(self, v):
        msg = Float64()
        msg.data = float(v)
        self.pub_vel.publish(msg)


# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = APFPlannerTrack()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()