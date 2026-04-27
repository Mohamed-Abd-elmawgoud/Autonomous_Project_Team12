# #!/usr/bin/env python3
# """
# ROS2 Node: Autonomous_Systems_MS_3_CLR_Alg_2_Lateral_Team_12
# ==============================================================
# Project  : Autonomous Systems – Milestone 3
# Algorithm: CLR Algorithm 2 – Lateral Control
# Team     : 12
# """

# import rclpy
# from rclpy.node import Node

# # Message types – adjust imports to match your actual message packages
# from std_msgs.msg import Float64
# from geometry_msgs.msg import Twist
# # from nav_msgs.msg import Odometry        # uncomment if needed
# # from sensor_msgs.msg import LaserScan    # uncomment if needed


# class AutonomousSystemsMS3CLRAlg2LateralTeam12(Node):
#     """
#     CLR Algorithm 2 – Lateral Control Node
#     ----------------------------------------
#     Subscribes to the current lateral error / state and publishes
#     the lateral control command (e.g. steering angle or angular velocity).

#     Topics
#     ------
#     Subscribed:
#         /lateral_error   (std_msgs/Float64)  – signed lateral deviation [m]

#     Published:
#         /cmd_vel         (geometry_msgs/Twist) – velocity command
#         /steering_angle  (std_msgs/Float64)    – raw steering output [rad]

#     Parameters
#     ----------
#     k_p  : float  – proportional gain  (default 1.0)
#     k_d  : float  – derivative gain    (default 0.1)
#     k_i  : float  – integral gain      (default 0.01)
#     max_steering : float – saturation limit [rad] (default 0.5)
#     """

#     def __init__(self) -> None:
#         super().__init__("Autonomous_Systems_MS_3_CLR_Alg_2_Lateral_Team_12")

#         # ── Declare & read parameters ────────────────────────────────────────
#         self.declare_parameter("k_p", 1.0)
#         self.declare_parameter("k_d", 0.1)
#         self.declare_parameter("k_i", 0.01)
#         self.declare_parameter("max_steering", 0.5)

#         self.k_p: float = self.get_parameter("k_p").value
#         self.k_d: float = self.get_parameter("k_d").value
#         self.k_i: float = self.get_parameter("k_i").value
#         self.max_steering: float = self.get_parameter("max_steering").value

#         # ── Internal PID state ───────────────────────────────────────────────
#         self._prev_error: float = 0.0
#         self._integral:   float = 0.0
#         self._prev_time = self.get_clock().now()

#         # ── Publishers ───────────────────────────────────────────────────────
        
#         self._pub_steering = self.create_publisher(
#             Float64, "/steering_angle", 10
#         )

#         # ── Subscribers ──────────────────────────────────────────────────────
#         self._sub_lateral_error = self.create_subscription(
#             Float64,
#             "/lateral_error",
#             self._lateral_error_callback,
#             10,
#         )

#         self.get_logger().info(
#             f"[Team 12] CLR Alg-2 Lateral node started. "
#             f"Kp={self.k_p}, Kd={self.k_d}, Ki={self.k_i}, "
#             f"max_steer={self.max_steering} rad"
#         )

#     # ── Callback ─────────────────────────────────────────────────────────────

#     def _lateral_error_callback(self, msg: Float64) -> None:
#         """Compute PID lateral control and publish commands."""
#         error: float = msg.data

#         # Time delta
#         now = self.get_clock().now()
#         dt: float = (now - self._prev_time).nanoseconds * 1e-9
#         if dt <= 0.0:
#             dt = 1e-3  # guard against zero division
#         self._prev_time = now

#         # PID terms
#         derivative: float  = (error - self._prev_error) / dt
#         self._integral     += error * dt
#         self._prev_error    = error

#         raw_steering: float = (
#             self.k_p * error
#             + self.k_d * derivative
#             + self.k_i * self._integral
#         )

#         # Saturate output
#         steering: float = max(
#             -self.max_steering,
#             min(self.max_steering, raw_steering)
#         )

#         # Publish steering angle
#         steer_msg = Float64()
#         steer_msg.data = steering
#         self._pub_steering.publish(steer_msg)

#         # Publish Twist (zero linear velocity – adjust as needed)
#         twist_msg = Twist()
#         twist_msg.linear.x  = 0.0   # set desired forward speed here
#         twist_msg.angular.z = steering
#         self._pub_cmd_vel.publish(twist_msg)

#         self.get_logger().debug(
#             f"error={error:.4f}  steering={steering:.4f}  "
#             f"integral={self._integral:.4f}"
#         )

#     # ── Helpers ──────────────────────────────────────────────────────────────

#     def reset_pid(self) -> None:
#         """Reset integrator and derivative memory."""
#         self._prev_error = 0.0
#         self._integral   = 0.0
#         self._prev_time  = self.get_clock().now()
#         self.get_logger().info("[Team 12] PID state reset.")


# # ── Entry point ───────────────────────────────────────────────────────────────

# def main(args=None) -> None:
#     rclpy.init(args=args)
#     node = AutonomousSystemsMS3CLRAlg2LateralTeam12()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("[Team 12] Shutting down lateral control node.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()




# # TO BE EDITED UPON
# #!/usr/bin/env python3
# """
# ROS2 Node: Autonomous_Systems_MS_3_CLR_Alg_2_Lateral_Team_12
# ==============================================================
# Project  : Autonomous Systems – Milestone 3
# Algorithm: CLR Algorithm 2 – Lateral Control
# Team     : 12
# """

# import rclpy
# from rclpy.node import Node

# # Message types – adjust imports to match your actual message packages
# from std_msgs.msg import Float64
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import JointState
# # from nav_msgs.msg import Odometry        # uncomment if needed
# # from sensor_msgs.msg import LaserScan    # uncomment if needed


# class AutonomousSystemsMS3CLRAlg2LateralTeam12(Node):
#     """
#     CLR Algorithm 2 – Lateral Control Node
#     ----------------------------------------
#     Subscribes to the current lateral error / state and publishes
#     the lateral control command (e.g. steering angle or angular velocity).

#     Topics
#     ------
#     Subscribed:
#         /lateral_error   (std_msgs/Float64)  – signed lateral deviation [m]

#     Published:
#         /cmd_vel         (geometry_msgs/Twist) – velocity command
#         /steering_angle  (std_msgs/Float64)    – raw steering output [rad]

#     Parameters
#     ----------
#     k_p  : float  – proportional gain  (default 1.0)
#     k_d  : float  – derivative gain    (default 0.1)
#     k_i  : float  – integral gain      (default 0.01)
#     max_steering : float – saturation limit [rad] (default 0.5)
#     """

#     def __init__(self) -> None:
#         super().__init__("Autonomous_Systems_MS_3_CLR_Alg_2_Lateral_Team_12")

#         # ── Declare & read parameters ────────────────────────────────────────
#         self.declare_parameter("k_p", 1.0)
#         self.declare_parameter("k_d", 0.1)
#         self.declare_parameter("k_i", 0.01)
#         self.declare_parameter("max_steering", 0.5)

#         self.Kp = 2.0
#         self.max_steering = 0.9

#         # ── Internal PID state ───────────────────────────────────────────────
#         self._prev_error: float = 0.0
#         self._integral:   float = 0.0
#         self._prev_time = self.get_clock().now()

#         # ── Publishers ───────────────────────────────────────────────────────
        
#         self._pub_steering = self.create_publisher(
#             Float64, "/steering_angle", 10
#         )

#         # ── Subscribers ──────────────────────────────────────────────────────
#         # self._sub_lateral_error = self.create_subscription(
#         #     Float64,
#         #     "/lateral_error",
#         #     self._lateral_error_callback,
#         #     10,
#         # )
#         self.joint_states_subscriber = self.create_subscription(
#             JointState,
#             '/joint_states',
#             self.joint_states_callback,
#             10
#         )

#         self.get_logger().info(
#             f"[Team 12] CLR Alg-2 Lateral node started. "
#             f"Kp={self.k_p}, Kd={self.k_d}, Ki={self.k_i}, "
#             f"max_steer={self.max_steering} rad"
#         )

#     # ── Callback ─────────────────────────────────────────────────────────────

#     def _lateral_error_callback(self, msg: JointState) -> None:
#         """Compute PID lateral control and publish commands."""
#         error: float = msg.data

#         # Time delta
#         now = self.get_clock().now()
#         dt: float = (now - self._prev_time).nanoseconds * 1e-9
#         if dt <= 0.0:
#             dt = 1e-3  # guard against zero division
#         self._prev_time = now

#         # PID terms
#         derivative: float  = (error - self._prev_error) / dt
#         self._integral     += error * dt
#         self._prev_error    = error

#         raw_steering: float = (
#             self.k_p * error
#             + self.k_d * derivative
#             + self.k_i * self._integral
#         )

#         # Saturate output
#         steering: float = max(
#             -self.max_steering,
#             min(self.max_steering, raw_steering)
#         )

#         # Publish steering angle
#         steer_msg = Float64()
#         steer_msg.data = steering
#         self._pub_steering.publish(steer_msg)

#         # Publish Twist (zero linear velocity – adjust as needed)
#         twist_msg = Twist()
#         twist_msg.linear.x  = 0.0   # set desired forward speed here
#         twist_msg.angular.z = steering
#         self._pub_cmd_vel.publish(twist_msg)

#         self.get_logger().debug(
#             f"error={error:.4f}  steering={steering:.4f}  "
#             f"integral={self._integral:.4f}"
#         )




# # ── Entry point ───────────────────────────────────────────────────────────────

# def main(args=None) -> None:
#     rclpy.init(args=args)
#     node = AutonomousSystemsMS3CLRAlg2LateralTeam12()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("[Team 12] Shutting down lateral control node.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()



#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import math
from rclpy.time import Time

class AutonomousSystemsMS3CLRAlg2LateralTeam12(Node):
    def __init__(self) -> None:
        super().__init__("Autonomous_Systems_MS_3_CLR_Alg_2_Lateral_Team_12")

        # Parameters
        self.declare_parameter("k_p", 2.0)
        self.declare_parameter("max_steering", 0.9)
        
        self.k_p = self.get_parameter("k_p").value
        self.max_steering = self.get_parameter("max_steering").value

        # 1. Declare the parameter with a default (in case the launch file doesn't provide one)
        self.declare_parameter("target_lateral_pos", 0.0)

        # 2. Retrieve the value
        self.target_lateral_pos = self.get_parameter("target_lateral_pos").value

        
        # Internal State
        self.current_steering_pos = 0.0

        self.desired_heading = 0.0

        self.current_heading = 0.0


        # Publishers
        self._pub_steering = self.create_publisher(Float64, "/steering_angle", 10)
        

        # Subscribers
        # Subscribing to joint_states to get the feedback from Gazebo
        self.joint_states_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info("[Team 12] Lateral node started using JointState feedback.")

    def odom_callback(self, msg: Odometry):
        # The orientation is a Quaternion
        q = msg.pose.pose.orientation
        
        # Standard formula to get Yaw (Heading) from Quaternion
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_heading = math.atan2(siny_cosp, cosy_cosp)


    def joint_states_callback(self, msg: JointState) -> None:
        """Extracts the average steering position from the front wheels."""
        # Check if 5 seconds have passed
        if not self.delay_passed:
            elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed_time < self.delay_duration:
                # Still waiting... don't publish anything yet
                return
            else:
                self.delay_passed = True
                self.get_logger().info("5 second delay finished. Starting publication.")
        try:
            # Dynamically find the indices of the steering joints
            names = msg.name
            fl_idx = names.index('front_left_steering_joint')
            fr_idx = names.index('front_right_steering_joint')

            # Extract positions
            fl_pos = msg.position[fl_idx]
            fr_pos = msg.position[fr_idx]

            # The center steering position is the average of both
            self.current_steering_pos = (fl_pos + fr_pos) / 2.0
            
            # Now trigger your control logic using this extracted position
            # For example, if you are calculating lateral error elsewhere:
            # self.compute_control(self.current_steering_pos)

        except ValueError as e:
            self.get_logger().warn(f"Steering joints not found in /joint_states: {e}")

        if len(msg.velocity) == 0:
            self.get_logger().warn('Received JointState with no velocity data.')
            return

        joint_names = list(msg.name)

        if 'front_left_steering_joint' in joint_names and 'front_right_steering_joint' in joint_names:
            idx_left  = joint_names.index('front_left_steering_joint')
            idx_right = joint_names.index('front_right_steering_joint')

            vel_left  = msg.velocity[idx_left]
            vel_right = msg.velocity[idx_right]

            # Average the two front wheels
            self.front_wheel_velocity = ((vel_left + vel_right) / 2.0) * 0.3  # rad/s → m/s

            self.get_logger().info(
                f'Front wheel velocities — left: {vel_left:.3f} rad/s, right: {vel_right:.3f} rad/s, avg: {self.front_wheel_velocity:.3f} m/s'
            )
        else:
            self.get_logger().warn('Front wheel joints not found in JointState message.')    

    def lateral_error_callback(self, msg: Float64) -> None:
        """
        Example of how you might use the extracted steering position 
        in conjunction with a lateral error message.
        """
        error_lateral_dis = self.target_lateral_pos - self.current_steering_pos

        error_heading = self.desired_heading - self.current_heading  # Placeholder for heading error

        
        # Simple P control example
        tan_num = (self.k_p * error_lateral_dis)/self.front_wheel_velocity if self.front_wheel_velocity > 0 else 0.0

        steering_cmd = math.atan(tan_num) + error_heading
        # Saturation
        steering_cmd = max(-self.max_steering, min(self.max_steering, steering_cmd))

        # Publish
        out_msg = Float64()
        out_msg.data = steering_cmd
        self._pub_steering.publish(out_msg)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutonomousSystemsMS3CLRAlg2LateralTeam12()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()