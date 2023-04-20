## Collision package utilising the laser scanner
## Can only detect objects that are on the floor
## Will not deal with object propped up, like benches, tables etc

import rclpy
from rclpy.node import Node


from geometry_msgs.msg import Twist
# sensor_msgs/msg/LaserScan - Laser scanner for collision detection
from sensor_msgs.msg import LaserScan

class stopColl(Node):
    def __init__(self):
        super().__init__('stop_collisions') # calls super's initialiser (so Node init method) and names node

        self.safe_distance = 0.8
        
        self.subscription = self.create_subscription(
            LaserScan,
            'scan', # subscribes to the scan topic
            self.callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.publisher_ = self.create_publisher(Twist, 'collision_flag', 10) # creates publisher


    def callback(self, msg):
        move = Twist()
        collision = False # Bool for if msg shpuld be published

        # frontClear = msg.ranges[0] > self.safe_distance and msg.ranges[30] > self.safe_distance and msg.ranges[330] > self.safe_distance

        # If any sensors around the front of the robot detect anything then
        # it will spin
        frontClear = msg.ranges[0] > self.safe_distance # Bool value
        for safety in range(50):
            frontClear = frontClear and msg.ranges[safety] > self.safe_distance
            frontClear = frontClear and msg.ranges[-safety] > self.safe_distance
        
        # Will loop over all ranges with a step of 30 
        for r in range(0, len(msg.ranges), 30):
            # If there is something within range but front is clear, go forwards
            if (msg.ranges[r] < self.safe_distance and frontClear):
                print(f"Seems to be clear in front")
                move.linear.x = 0.25
                move.angular.z = 0.0
                collision = True
            # If something is close to robot then spin
            elif (msg.ranges[r] < self.safe_distance or frontClear == False):
                print(f"Collision at laser point {r}")
                move.linear.x = 0.0
                collision = True

                # If object is on left turn right, etc
                # Only attempts as will turn right when both left and right below threshold
                if r >= 180 and r < 400: # 225
                    move.angular.z = 0.3
                elif r >= 0 and r < 180: # 135
                    move.angular.z = -0.3
                else:
                    move.angular.z = -0.3


        # output message when a condition has been met
        if collision == True:
            self.publisher_.publish(move)
    


def main(args=None):
    print('Starting collisions system.')
    rclpy.init(args=args)

    node_ = stopColl()

    rclpy.spin(node_)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()