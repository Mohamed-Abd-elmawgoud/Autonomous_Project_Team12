import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class OLRNode(Node):

    def __init__(self):
        super().__init__('olr_node')

        # Subscriber to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Publishers
        self.velocity_pub = self.create_publisher(Float64, '/velocity', 10)
        self.steering_pub = self.create_publisher(Float64, '/steering_angle', 10)

        # Store latest joint state
        self.latest_joint_state = None

        # Motion commands (constant for circular motion)
        self.velocity = 10.0
        self.steering_angle = 0.5

        # Timer for printing joint states (every 3 seconds)
        # self.print_timer = self.create_timer(3.0, self.print_joint_states)

        # Timer for publishing commands (every 0.1 seconds)
        self.command_timer = self.create_timer(0.1, self.publish_commands)

        self.get_logger().info("Circular Vehicle Controller started")

    def joint_callback(self, msg):
        self.latest_joint_state = msg

    def print_joint_states(self):
        if self.latest_joint_state is not None:
            positions = self.latest_joint_state.position
            velocities = self.latest_joint_state.velocity
            self.get_logger().info(f'Joint positions: {positions}')
            self.get_logger().info(f'Joint velocities: {velocities}')
        else:
            self.get_logger().info("Waiting for joint states...")

    def publish_commands(self):

        vel_msg = Float64()
        vel_msg.data = self.velocity
        self.velocity_pub.publish(vel_msg)

        steer_msg = Float64()
        steer_msg.data = self.steering_angle
        self.steering_pub.publish(steer_msg)


def main(args=None):
    rclpy.init(args=args)

    node = OLRNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()