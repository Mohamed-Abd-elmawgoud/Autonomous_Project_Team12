import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


class track2_planner(Node):

    def __init__(self):
        super().__init__('track2_planner')

        # Publisher: /velocity (rad/s to the wheel controller)
        self.desired_velocity_publisher = self.create_publisher(
            Float64,
            '/desired_velocity',
            10
        )
        # Publisher: /lane (m to the lane controller)
        self.desired_lane_publisher = self.create_publisher(
            Float64,
            '/desired_lane',
            10
        )
         # Subscriber: odometry — gives body_link speed in m/s directly
        self.odom_subscriber = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )
           # This allows the launch file to override the default 2.0
        self.declare_parameter('desired_velocity', 2.0)

        # 2. Retrieve the parameter value
        self.desired_velocity = self.get_parameter('desired_velocity').value
        self.desired_lane = 0.0
        self.current_x = 0.0
        # Add this — you'll see it in the terminal on startup
        self.get_logger().info(f'desired_velocity set to: {self.desired_velocity}')
        self.get_logger().info(f'desired_lane set to: {self.desired_lane}')

        self.sample_time = 0.1       # seconds (matches 10 Hz timer)

     

        # Timer: publish at 10 Hz
        self.timer = self.create_timer(self.sample_time, self.timer_callback)

        self.get_logger().info('Node started — reading speed from odometry.')
    def odom_callback(self, msg: Odometry):
        """Read the vehicle's linear speed directly from odometry (m/s)."""
        self.current_x = msg.pose.pose.position.x

        self.get_logger().info(
            f'Odometry position: ({self.current_x:.3f}'
        )

    def timer_callback(self):
        if self.current_x >= 0.0 and self.current_x < 1.5:
            self.desired_velocity = 2.0
            self.desired_lane = 0.1875
        elif self.current_x >= 1.5 and self.current_x < 4.2:
            self.desired_velocity = 0.1
            self.desired_lane = -0.1875
        elif self.current_x >= 4.2 and self.current_x < 6.0:
            self.desired_velocity = 2.0
            self.desired_lane = -0.1875
        elif self.current_x >= 6.0 and self.current_x < 8.2:
            self.desired_velocity = 0.1
            self.desired_lane = 0.1875
        elif self.current_x >= 8.2 and self.current_x < 9.5:
            self.desired_velocity = 2.0
            self.desired_lane = 0.1875
        elif self.current_x >= 9.5:
            self.desired_velocity = 0.0
            self.desired_lane = 0.1875    


        msg = Float64()
        msg.data = self.desired_velocity
        self.desired_velocity_publisher.publish(msg)

        msg2 = Float64()
        msg2.data = self.desired_lane
        self.desired_lane_publisher.publish(msg2)


def main(args=None):
    rclpy.init(args=args)
    node = track2_planner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()