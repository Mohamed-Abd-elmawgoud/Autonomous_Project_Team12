#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import math


class StanleyController(Node):

    def __init__(self):
        super().__init__("stanley_fixed")

        # ---------------- PARAMETERS ----------------
        self.declare_parameter("k_p", 1.0)
        self.declare_parameter("max_steering", 0.6108)
        self.declare_parameter("target_lateral_pos", 1.0)
        self.declare_parameter("lane_heading", 0.0)

        self.k_p = self.get_parameter("k_p").value
        self.max_steering = self.get_parameter("max_steering").value
        # self.target_lateral_pos = self.get_parameter("target_lateral_pos").value
        self.lane_heading = self.get_parameter("lane_heading").value

        # ---------------- STATE ----------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.speed = 0.0

        # ---------------- IO ----------------
        self.pub = self.create_publisher(Float64, "/steering_angle", 10)
        self.create_subscription(Odometry, "/odom", self.cb, 10)
          # Subscriber: odometry — gives body_link speed in m/s directly
        self.desired_lane_subscriber = self.create_subscription(
            Float64,
            "/desired_lane",
            self.desired_lane_callback,
            10
        )


        self.get_logger().info("Stanley FIXED started")

    # =========================================================
    # ODOM CALLBACK
    # =========================================================
    def cb(self, msg: Odometry):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

        self.speed = msg.twist.twist.linear.x

        self.compute()

    def desired_lane_callback(self, msg: Float64):
        self.target_lateral_pos = msg.data

    # =========================================================
    # STANLEY CONTROL
    # =========================================================
    def compute(self):

        # ---------------- SPEED SAFETY ----------------
        v = abs(self.speed)  # IMPORTANT FIX (removes reverse issues)
        v = max(0.5, v)

        # ---------------- CROSS TRACK ERROR ----------------
        cte = self.target_lateral_pos - self.y

        # ---------------- HEADING ERROR ----------------
        heading_error = self.lane_heading - self.yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        # ---------------- STANLEY ----------------
        stanley = math.atan2(self.k_p * cte, v)

        steer = heading_error + stanley

        # clamp
        steer = max(-self.max_steering, min(self.max_steering, steer))

        # ---------------- OUTPUT ----------------
        msg = Float64()
        msg.data = steer
        self.pub.publish(msg)

        # ---------------- DEBUG ----------------
        # self.get_logger().info(
        #     f"[STANLEY] y={self.y:.2f} "
        #     f"cte={cte:.2f} "
        #     f"yaw={math.degrees(self.yaw):.1f} "
        #     f"steer={math.degrees(steer):.1f}"
        # )


def main():
    rclpy.init()
    node = StanleyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()