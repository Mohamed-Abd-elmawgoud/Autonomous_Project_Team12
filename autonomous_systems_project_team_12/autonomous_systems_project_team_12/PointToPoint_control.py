import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import math

class PointToPoint_control(Node):

    def __init__(self):
        super().__init__('PointToPoint_control')

        # Publisher: Outputs linear velocity (m/s)
        self.velocity_publisher = self.create_publisher(Float64, '/velocity', 10)

        # Subscriber: Odometry for current position (x, y)
        self.odom_subscriber = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10)

        # Subscriber: Target coordinates (Goal)
        self.goal_subscriber = self.create_subscription(
            Point, "/goal_point", self.goal_callback, 10)

        # --- PID Parameters for Distance ---
        # These need tuning! Kp acts as the "approach speed" factor.
        self.Kp = 1.0
        self.Ki = 0.01
        self.Kd = 0.1

        # --- State Variables ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0
        
        self.prev_dist_error = 0.0
        self.integral_dist_error = 0.0
        self.sample_time = 0.1  # 10 Hz
        
        # Threshold to consider the goal "reached" (meters)
        self.goal_tolerance = 0.1

        self.timer = self.create_timer(self.sample_time, self.timer_callback)
        self.get_logger().info('Point-to-Point Controller Node Started.')

    def odom_callback(self, msg: Odometry):
        """Update the current 2D position of the robot."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def goal_callback(self, msg: Point):
        """Update the target destination."""
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.get_logger().info(f'New Goal Received: x={self.goal_x}, y={self.goal_y}')

    def timer_callback(self):
        dist_error = math.sqrt(
            (self.goal_x - self.current_x)**2 +
            (self.goal_y - self.current_y)**2
        )

        if dist_error < self.goal_tolerance:
            velocity_cmd = 0.0
            self.integral_dist_error = 0.0
        else:
            P_out = self.Kp * dist_error
            self.integral_dist_error += dist_error * self.sample_time
            I_out = self.Ki * self.integral_dist_error
            D_out = self.Kd * (dist_error - self.prev_dist_error) / self.sample_time
            velocity_cmd = P_out + I_out + D_out

            # ── Slow down when approaching the goal (corner) ──────────────────
            if dist_error < 0.6:              # within 1.2m of waypoint = corner zone
                max_vel = 0.5                 # slow speed through corners
            else:
                max_vel = 1.0                 # full speed on straights

            velocity_cmd = min(velocity_cmd, max_vel)

        msg = Float64()
        msg.data = velocity_cmd
        self.velocity_publisher.publish(msg)
        self.prev_dist_error = dist_error

def main(args=None):
    rclpy.init(args=args)
    node = PointToPoint_control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():              # ← same fix
            rclpy.shutdown()