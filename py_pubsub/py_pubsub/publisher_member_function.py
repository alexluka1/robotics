import rclpy
from rclpy.node import Node

# String message 
from std_msgs.msg import String


###
# Creating the publishing node
# Uses init to set up node features such as how to publish, a timer 

# Create a method to make a string message with "Hello world {i}",
# every tick of the timer

# Think the last bit just logs the msg data
###
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher') # calls super's initialiser (so Node init method) and names node
        self.publisher_ = self.create_publisher(String, 'topic', 10) # creates publisher
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # creates timer
        self.i = 0

    def timer_callback(self):
        msg = String() # msg type
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg) # pushlishes msg
        self.get_logger().info('Publishing: "%s"' % msg.data) # logs msg (adds extra string info)
        self.i += 1


###
# Creates the ros2 publisher node and starts it (spin)
# Destroys node once script has ended
###
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()