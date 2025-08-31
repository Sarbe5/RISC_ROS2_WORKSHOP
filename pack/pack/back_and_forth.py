import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MyNode(Node):

    def __init__(self):
        super().__init__("move_st")
        self.publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        
        # initial velocity
        self.vel = 1.0  

        # timer: runs every 0.5 sec
        self.timer = self.create_timer(0.5, self.send_velocity)

        # counter to decide when to switch direction
        self.counter = 0  

    def send_velocity(self):
        msg = Twist()
        msg.linear.x = self.vel
        self.publisher.publish(msg)

        self.counter += 1

        # after 10 timer calls (~5 seconds), reverse direction
        if self.counter >= 10:  
            self.vel = -self.vel  # toggle direction
            self.counter = 0      # reset counter


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
