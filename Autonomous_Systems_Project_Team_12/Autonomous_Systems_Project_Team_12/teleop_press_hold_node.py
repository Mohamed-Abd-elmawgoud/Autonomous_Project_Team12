import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

import threading
import curses
import time


class ToggleSteeringTeleop(Node):
    def __init__(self):
        super().__init__('toggle_steering_teleop')

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )

        # Publishers
        self.vel_pub = self.create_publisher(Float64, '/velocity', 10)
        self.steer_pub = self.create_publisher(Float64, '/steering_angle', 10)

        # Vehicle state
        self.latest_joint_state = None
        self.current_velocity = 0.0
        self.current_steering = 0.0

        # Motion values
        self.vel_value = 10.0
        self.steer_value = 0.5

        # Toggle flags
        self.left_steer_on = False
        self.right_steer_on = False

        # Set of currently pressed keys for forward/back
        self.pressed_keys = set()

        # Timer: print joint states every 2 seconds
        # self.print_timer = self.create_timer(2.0, self.print_joint_states)

        # Timer: publish commands at 10 Hz
        self.command_timer = self.create_timer(0.1, self.publish_commands)w

        # Keyboard thread
        self.thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.thread.start()

        self.get_logger().info("Toggle-steering teleop started")
        self.get_logger().info("Controls: w/s -> forward/back, a/d -> toggle left/right steering, q -> quit")

    def joint_callback(self, msg):
        self.latest_joint_state = msg

    # def print_joint_states(self):
    #     if self.latest_joint_state:
    #         # self.get_logger().info(f'Joint positions: {self.latest_joint_state.position}')
    #     else:
    #         # self.get_logger().info('Waiting for joint states...')

    def publish_commands(self):
        # Determine velocity
        velocity = 0.0
        if 'w' in self.pressed_keys:
            velocity += self.vel_value
        if 's' in self.pressed_keys:
            velocity -= self.vel_value
        self.current_velocity = velocity

        # Determine steering from toggle flags
        steering = 0.0
        if self.left_steer_on:
            steering += self.steer_value
        if self.right_steer_on:
            steering -= self.steer_value
        self.current_steering = steering

        # Publish
        vel_msg = Float64()
        vel_msg.data = self.current_velocity
        self.vel_pub.publish(vel_msg)

        steer_msg = Float64()
        steer_msg.data = self.current_steering
        self.steer_pub.publish(steer_msg)

    def keyboard_loop(self):
        stdscr = curses.initscr()
        curses.cbreak()
        stdscr.keypad(True)
        stdscr.nodelay(True)

        try:
            while rclpy.ok():
                key = stdscr.getch()

                if key != -1:
                    if key == ord('w'):
                        self.pressed_keys.add('w')
                    elif key == ord('s'):
                        self.pressed_keys.add('s')
                    elif key == ord('a'):
                        self.left_steer_on = not self.left_steer_on
                        self.get_logger().info(f"Left steering toggled {'ON' if self.left_steer_on else 'OFF'}")
                    elif key == ord('d'):
                        self.right_steer_on = not self.right_steer_on
                        self.get_logger().info(f"Right steering toggled {'ON' if self.right_steer_on else 'OFF'}")
                    elif key == ord('q'):
                        self.get_logger().info("Exiting teleop...")
                        rclpy.shutdown()
                        break
                else:
                    # Key release detection for forward/backward
                    self.pressed_keys.clear()

                time.sleep(0.05)

        finally:
            curses.nocbreak()
            stdscr.keypad(False)
            curses.echo()
            curses.endwin()


def main(args=None):
    rclpy.init(args=args)
    node = ToggleSteeringTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()