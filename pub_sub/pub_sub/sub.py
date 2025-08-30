import rclpy 
from rclpy.node import Node 
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__ (self):
        super().__init__('subscriber_node')  #name of the node
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')  #logs the received message to the console                     

def main(args=None):
    rclpy.init(args=args)  #initializes the ROS2 python client library
    subscriber_node = SubscriberNode() #creates an instance of the SubscriberNode class
    rclpy.spin(subscriber_node)  #keeps the node running and processing callbacks
    subscriber_node.destroy_node()  #destroys the node when done
    rclpy.shutdown()  #shuts down the ROS2 python client library

if __name__ == '__main__':
    main()