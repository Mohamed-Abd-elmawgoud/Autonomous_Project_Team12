# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64
# from sensor_msgs.msg import JointState


# class Autonomous_Systems_MS_3_CLR_Alg_1_Speed_Team_12(Node):

#     def __init__(self):
#         super().__init__('Autonomous_Systems_MS_3_CLR_Alg_1_Speed_Team_12')

#         # Publisher: /velocity
#         self.velocity_publisher = self.create_publisher(
#             Float64,
#             '/velocity',
#             10
#         )

#         # Subscriber: /joint_states
#         self.joint_states_subscriber = self.create_subscription(
#             JointState,
#             '/joint_states',
#             self.joint_states_callback,
#             10
#         )

#         # Timer: publish at 10 Hz
#         self.timer = self.create_timer(0.1, self.timer_callback)

#         self.get_logger().info('Node started.')

#     def joint_states_callback(self, msg: JointState):
#         """Callback triggered when a JointState message is received on /joint_states."""
#         self.get_logger().info(
#             f'Received JointState — names: {msg.name}, positions: {msg.position}'
#         )
#         # TODO: Process the incoming JointState message here
#         velocity_curr = msg.velocity

#     def timer_callback(self):
#         """Publishes a Float64 message on /velocity at 10 Hz."""
#         msg = Float64()
#         msg.data = 0.0  # TODO: Set the desired velocity value here

#         self.velocity_publisher.publish(msg)
#         self.get_logger().debug(f'Published velocity: {msg.data}')


# def main(args=None):
#     rclpy.init(args=args)

#     node = Autonomous_Systems_MS_3_CLR_Alg_1_Speed_Team_12()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


class Autonomous_Systems_MS_3_CLR_Alg_1_Speed_Team_12(Node):

    def __init__(self):
        super().__init__('Autonomous_Systems_MS_3_CLR_Alg_1_Speed_Team_12')

        # Publisher: /velocity
        self.velocity_publisher = self.create_publisher(
            Float64,
            '/velocity',
            10
        )

        # Subscriber: /joint_states
        self.joint_states_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        self.Kp = 3.0  # Proportional gain for the controller
        self.desired_velocity = 2.0  # Desired velocity (m/s)
        self.sample_time = 0.1  # Sample time (s)

        # Timer: publish at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Store the average velocity of the rear wheels (rad/s)
        self.rear_wheel_velocity = 0.0

        self.get_logger().info('Node started.')

    def joint_states_callback(self, msg: JointState):
        """Callback triggered when a JointState message is received on /joint_states."""
        if len(msg.velocity) == 0:
            self.get_logger().warn('Received JointState with no velocity data.')
            return

        joint_names = list(msg.name)

        if 'rear_left_wheel_joint' in joint_names and 'rear_right_wheel_joint' in joint_names:
            idx_left  = joint_names.index('rear_left_wheel_joint')
            idx_right = joint_names.index('rear_right_wheel_joint')

            vel_left  = msg.velocity[idx_left]
            vel_right = msg.velocity[idx_right]

            # Average the two rear wheels
            #self.rear_wheel_velocity = (vel_left + vel_right) / 2.0
            self.rear_wheel_velocity = ((vel_left + vel_right) / 2.0) * 0.3  # rad/s → m/s

            self.get_logger().info(
                f'Rear wheel velocities — left: {vel_left:.3f} rad/s, right: {vel_right:.3f} rad/s, avg: {self.rear_wheel_velocity:.3f} m/s'
            )
        else:
            self.get_logger().warn('Rear wheel joints not found in JointState message.')

    def timer_callback(self):
        """Publishes the average rear wheel angular velocity (rad/s) on /velocity at 10 Hz."""
        msg = Float64()
        
        velocity_curr = self.rear_wheel_velocity
        accel = self.Kp * (velocity_curr - self.desired_velocity)

        velocity_cmd = velocity_curr + accel * self.sample_time
        msg.data = velocity_cmd

        self.velocity_publisher.publish(msg)
        self.get_logger().debug(f'Published velocity: {msg.data}')


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