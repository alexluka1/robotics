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
        # Spin robot around in around 360
        if self.count < 98:
            self.count += 1
            self.publisher_.publish(msg)  
        # Forwards 
        else:
            msg.angular.z = 0.0
            msg.linear.x = 0.25
            self.publisher_.publish(msg) 



        ## Attempt to turn the correct way by taking the minimum of the msg.ranges
        # minDistR = math.inf # Sets min distance to inf
        # minDistL = math.inf # Sets min distance to inf

        # for safety in range(50):
        #     # frontClear = frontClear and msg.ranges[safety] > self.safe_distance
        #     # frontClear = frontClear and msg.ranges[-safety] > self.safe_distance

        #     # Find minimum ranges distance for R and L
        #     if minDistR > msg.ranges[safety]:
        #         minDistR = msg.ranges[safety]

        #     if minDistL > msg.ranges[-safety]:
        #         minDistL = msg.ranges[-safety]
        

        # minDist = math.inf
        # # If the left and right are too close then turn right
        # # stops loops
        # if minDistL - minDistR < 0.2:
        #     minDist = minDistR
        #     move.angular.z = -0.3
        #     move.linear.x = 0.0
        #     collision = True
        # elif minDistL < minDistR:
        #     minDist = minDistL
        #     move.angular.z = 0.3
        #     move.linear.x = 0.0
        #     collision = True
        # else:
        #     minDist = minDistR
        #     move.angular.z = -0.3
        #     move.linear.x = 0.0
        #     collision = True
            
        
        # if minDist < self.safe_distance:
        #     # print("HIT")
        #     move.linear.x = 0.0
        #     collision = True
        # else:
        #     # print("Seems to be clear in front")
        #     collision = False

            
        # if collision == True:
        #     self.publisher_.publish(move)

    


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