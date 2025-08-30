import rclpy 
from rclpy.node import Node 
from std_msgs.msg import String 

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')    #name of the node
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        '''
        creates the publisher with data type string, topic name topic and queue size 10
        queue size is the maximum number of messages to be queued for delivery to subscribers like suppose if the publisher is publishing messages faster than the subscriber can process them, the queue will store up to 10 messages before dropping the oldest ones. 
        When the 11th message goes the 1st message is dropped
        '''

        timer_period = 1.0   #time period(seconds) of calling the timer callback function
        self.timer = self.create_timer(timer_period, self.timer_callback) #creates a timer that calls the timer_callback function
        self.i = 0 #counter variable
        
    def timer_callback(self):
        msgl = String()
        msgl.data = f'Hello, this is message number {self.i}'  #creates the message to be published #fstring is used to format the string with the value of i
        self.publisher_.publish(msgl)  #publishes the message
        self.get_logger().info(f'Publishing: "{msgl.data}"')  #logs the published message to the console
        self.i += 1  #increments the counter variable                   
    
def main(args=None):
    rclpy.init(args=args)  #initializes the ROS2 python client library
    publisher_node = PublisherNode() #creates an instance of the PublisherNode class
    rclpy.spin(publisher_node)  #keeps the node running and processing callbacks
    publisher_node.destroy_node()  #destroys the node when done
    rclpy.shutdown()  #shuts down the ROS2 python client library            

if __name__ == '__main__':
    main()