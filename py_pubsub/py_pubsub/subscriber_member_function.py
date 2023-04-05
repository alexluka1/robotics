import rclpy
from rclpy.node import Node

from std_msgs.msg import String
## Dependencies are the same so nothing to add to config files

###
# Again inherits from Node
###
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        ## Changes to subscription
        ## Runs listener_callback when something is heard
        self.subscription = self.create_subscription(
            String,
            'topic', # subscribes to the topic topic
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    ## Method to run when something is heard
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()