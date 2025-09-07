import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int64


class mynode(Node):

    def __init__(self):
        super().__init__("listner")
        self.get_logger().info("listner active")
        self.c = 0
        self.subscription = self.create_subscription(
            String, "sentence", self.callback, 10)

    def callback(self, msg: String):
        self.get_logger().info(str(msg))


def main(args=None):
    rclpy.init(args=args)
    node = mynode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
