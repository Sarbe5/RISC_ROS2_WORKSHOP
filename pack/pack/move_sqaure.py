import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TurtleSquare(Node):
    def __init__(self):
        super().__init__('turtle_square')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.move_square)
        self.done = False

    def move_square(self):
        if self.done:
            return

        side_length = 2.0   # seconds of forward motion (adjust for distance)
        speed_linear = 2.0  # forward speed
        angular_speed = math.pi / 4  # rad/s = 45°/s
        relative_angle = math.pi / 2  # 90° turn

        twist = Twist()

        for i in range(4):
            # Move forward
            twist.linear.x = speed_linear
            twist.angular.z = 0.0
            t0 = time.time()
            while time.time() - t0 < side_length:
                self.publisher_.publish(twist)

            # Stop before turning
            twist.linear.x = 0.0
            self.publisher_.publish(twist)
            time.sleep(1.0)

            # Rotate 90 deg
            twist.angular.z = angular_speed
            t0 = time.time()
            while time.time() - t0 < relative_angle / angular_speed:
                self.publisher_.publish(twist)

            # Stop after turn
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            time.sleep(1.0)

        self.get_logger().info("Square path complete!")
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSquare()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
