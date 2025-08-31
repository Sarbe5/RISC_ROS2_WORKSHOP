import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int64

class mynode(Node):

    def __init__(self):
        super().__init__("talker")
        self.get_logger().info("talker active")
        self.counter = 0
        self.create_timer(1.0,self.timer_callback)
        self.publisher = self.create_publisher(String,"sentence",10)
        self.create_timer(1.0,self.callback)


    def timer_callback(self):
        self.get_logger().info(f"Hello there!! at {self.counter}, i am talking")
        self.counter +=1

    def callback(self):
        msg = String()
        msg.data = f"This msg is for listner at {self.counter}"
        #print(str(msg))
        self.publisher.publish(msg)

        

def main(args=None):
    rclpy.init(args=args)
    node = mynode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()