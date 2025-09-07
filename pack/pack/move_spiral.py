import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class mynode(Node):

    def __init__(self):
        super().__init__("move_spiral")
        self.sp = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.xt = 5.0
        self.timer = self.create_timer(0.5, self.sendv)

    def sendv(self):
        msg = Twist()
        msg.linear.x = self.xt
        self.xt -= 0.1
        msg.angular.z = 2.0
        self.sp.publish(msg)
        if (self.xt < 0):
            self.xt = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = mynode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
