### CORECT CODE BELOW 
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
        #self.Kp = 1.0  # Proportional gain for the controller

        self.WHEEL_RADIUS = 0.3  # meters, from URDF

        # --- PID Parameters ---
        self.Kp = 1.5   # Proportional gain
        self.Ki = 0.1   # Integral gain
        self.Kd = 0.05  # Derivative gain

        self.desired_velocity = 2.0  # Desired velocity (m/s)
        self.sample_time = 0.1  # Sample time (s)

        # --- PID State Variables ---
        self.integral_error = 0.0
        self.prev_error = 0.0
        self.rear_wheel_velocity = 0.0

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
        
        # velocity_curr = self.rear_wheel_velocity
        # accel = self.Kp * (self.desired_velocity - velocity_curr)

        # velocity_cmd = velocity_curr + accel * self.sample_time
        # msg.data = velocity_cmd

        # 1. Calculate current error
        error = self.desired_velocity - self.rear_wheel_velocity
        
        # 2. Proportional term
        P_out = self.Kp * error
        
        # 3. Integral term (accumulated error over time)
        self.integral_error += ((error + self.prev_error)/2) * self.sample_time
        I_out = self.Ki * self.integral_error
        
        # 4. Derivative term (change in error)
        derivative = (error - self.prev_error) / self.sample_time
        D_out = self.Kd * derivative

        
        # 5. Total control signal (Acceleration/Command Adjustment)
        accel_cmd = P_out + I_out + D_out
        
        # Update command based on current state + PID adjustment
        velocity_cmd = self.rear_wheel_velocity + (accel_cmd * self.sample_time)

          # ✅ Convert to rad/s before sending to the controller
        velocity_cmd_rads = velocity_cmd / self.WHEEL_RADIUS

        msg.data = velocity_cmd_rads
        self.velocity_publisher.publish(msg)

        self.prev_error = error
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


##### CORRECT CODE ENDS HERE    

