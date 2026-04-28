import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


class Autonomous_Systems_MS_3_CLR_Alg_1_Speed_Team_12(Node):

    def __init__(self):
        super().__init__('Autonomous_Systems_MS_3_CLR_Alg_1_Speed_Team_12')

        # Publisher: /velocity (rad/s to the wheel controller)
        self.velocity_publisher = self.create_publisher(
            Float64,
            '/velocity',
            10
        )

        # Subscriber: odometry — gives body_link speed in m/s directly
        self.odom_subscriber = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        # --- PID Parameters ---
        self.Kp = 1.5
        self.Ki = 0.1
        self.Kd = 0.05

        self.desired_velocity = 2.0  # m/s
        self.sample_time = 0.1       # seconds (matches 10 Hz timer)

        # --- PID State Variables ---
        self.integral_error = 0.0
        self.prev_error = 0.0
        self.current_velocity = 0.0  # m/s, updated by odometry

        # Timer: publish at 10 Hz
        self.timer = self.create_timer(self.sample_time, self.timer_callback)

        self.get_logger().info('Node started — reading speed from odometry.')

    def odom_callback(self, msg: Odometry):
        """Read the vehicle's linear speed directly from odometry (m/s)."""
        self.current_velocity = msg.twist.twist.linear.x

        self.get_logger().info(
            f'Odometry speed: {self.current_velocity:.3f} m/s'
        )

    def timer_callback(self):
        """PID controller: computes velocity command and publishes in rad/s."""

        # 1. Error in m/s
        error = self.desired_velocity - self.current_velocity

        # 2. Proportional term
        P_out = self.Kp * error

        # 3. Integral term (trapezoidal integration)
        self.integral_error += ((error + self.prev_error) / 2.0) * self.sample_time
        I_out = self.Ki * self.integral_error

        # 4. Derivative term
        derivative = (error - self.prev_error) / self.sample_time
        D_out = self.Kd * derivative

        # 5. Total acceleration command
        accel_cmd = P_out + I_out + D_out

        # 6. New desired speed in m/s
        velocity_cmd_ms = self.current_velocity + (accel_cmd * self.sample_time)

        # 7. Convert m/s → rad/s for the wheel velocity controller
        #velocity_cmd_rads = velocity_cmd_ms / self.WHEEL_RADIUS

        msg = Float64()
        msg.data = velocity_cmd_ms
        self.velocity_publisher.publish(msg)

        self.prev_error = error

        self.get_logger().debug(
            f'error: {error:.3f} | cmd: {velocity_cmd_ms:.3f} m/s '
        )


def main(args=None):
    rclpy.init(args=args)
    node = Autonomous_Systems_MS_3_CLR_Alg_1_Speed_Team_12()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()