import rclpy
from rclpy.node import Node

# Gets twist method to move robot
from geometry_msgs.msg import Twist


class MoveSquare(Node):
    def __init__(self):
        super().__init__('move_square') # calls super's initialiser (so Node init method) and names node
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) # creates publisher
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # creates timer

    def timer_callback(self):
        msg = Twist() # msg type
        ## Moves robot forward and turns it
        msg.linear.x = 1.0
        msg.angular.y = 1.0

        self.publisher_.publish(msg) # pushlishes msg
        ## cant get it working ## self.get_logger().info('Publishing: "%s"' % msg.linear) # logs msg (adds extra string info)



def main(args=None):
    rclpy.init(args=args)

    move_square = MoveSquare()

    rclpy.spin(move_square)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    move_square.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        
