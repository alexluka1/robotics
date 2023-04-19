## Node to just spin the robot every now and again

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class spinRobo(Node):

    def __init__(self):
        super().__init__('spinRobo')

        self.count = 0 # Count how many times a msg could have been published

        self.publisher_ = self.create_publisher(Twist, 'spin_vel', 10)

        # Timer to set flag to True allowing the robot to spin
        timer_period = 30  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback_flag)

        # Second timer to publish Twist messages
        timer_period2 = 0.1  # seconds
        self.timer2 = self.create_timer(timer_period2, self.timer_callback_spin)

    def timer_callback_flag(self):
        # msg = Twist()
        # msg.angular.z = 0.5
        print("Spinning")
        # Resets count to allow for more spinning
        self.count = 0
   

    def timer_callback_spin(self):
        msg = Twist()
        msg.angular.z = 1.0
        if self.count < 98:
            self.count += 1
            self.publisher_.publish(msg)  
        else:
            msg.angular.z = 0.0
            msg.linear.x = 0.25
            self.publisher_.publish(msg) 

    


def main(args=None):
    print('Starting spin system.')
    rclpy.init(args=args)

    node_ = spinRobo()

    rclpy.spin(node_)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()