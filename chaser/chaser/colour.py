#!/usr/bin/env python

# This code was adapted from the github page 
#https://github.com/LCAS/teaching/tree/lcas_humble/cmp3103m_ros2_code_fragments

# An example of TurtleBot 3 subscribe to camera topic, mask colours, find and display contours, and move robot to center the object in image frame
# Written for humble
# cv2 image types - http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np

# Detecting if within 1m
from sensor_msgs.msg import LaserScan


class ColourChaser(Node):
    def __init__(self):
        super().__init__('colour_chaser')
        
        self.turn_vel = 0.0
        self.speed = 0.0
        self.finding = False

        # -----------
        # Once a colour has been found this will be set to True
        # and all masks associated will be turned off 
        self.colourFinding = {"Yellow": False,
                         "Blue": False,
                         "Green": False,
                         "Red": False}
        # Used when determining if close enougth to a colour to say its found
        self.possibleFinding = {"Yellow": False,
                         "Blue": False,
                         "Green": False,
                         "Red": False} 
        # -----------

        # publish cmd_vel topic to move the robot
        self.pub_vel = self.create_publisher(Twist, 'colour_vel', 10)

        # create timer to publish cmd_vel topic
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # subscribe to the camera topic
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)


        # Subs to laser to check coloured obj is withing distance
        self.laser_subscription = self.create_subscription(
            LaserScan,
            'scan', # subscribes to the scan topic
            self.laser_callback,
            10)
        

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # -------- Ticking off list
        # self.finding_timer = self.create_timer(timer_period, self.timer_finding_callback)


    def laser_callback(self, msg):
        frontDist = msg.ranges[0]
        # Controls when the objects are found
        for findings in self.possibleFinding:
            if self.possibleFinding[findings] == True and frontDist < 1.0:
                self.colourFinding[findings] = True
                print(f"Found {findings}")
                print(self.colourFinding)

    

    
    
    def camera_callback(self, data):
        # -------- Resetting attributes
        for findings in self.possibleFinding:
            self.possibleFinding[findings] = False
        # -------- Resetting attributes
        
        
        #self.get_logger().info("camera_callback")

        cv2.namedWindow("Image window", 1)

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Convert image to HSV
        current_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        # Create mask for range of colours (HSV low values, HSV high values)
        #current_frame_mask = cv2.inRange(current_frame_hsv,(70, 0, 50), (150, 255, 255))
        current_frame_maskYellow = cv2.inRange(current_frame_hsv,(22, 100, 100), (30, 255, 255)) # Yellow
        current_frame_maskGreen = cv2.inRange(current_frame_hsv,(34,100,0), (86,255,255)) # Green
        current_frame_maskBlue = cv2.inRange(current_frame_hsv,(100,100,0), (130,255,255)) # Blue
        current_frame_maskRed = cv2.inRange(current_frame_hsv,(170,100,0), (180,255,255)) # Red
        current_frame_maskRed2 = cv2.inRange(current_frame_hsv,(0,100,0), (5,255,255)) # Red

        # Coloured contours
        contoursYellow, hierarchy = cv2.findContours(current_frame_maskYellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contoursGreen, hierarchy = cv2.findContours(current_frame_maskGreen, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contoursBlue, hierarchy = cv2.findContours(current_frame_maskBlue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contoursRed, hierarchy = cv2.findContours(current_frame_maskRed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contoursRed2, hierarchy = cv2.findContours(current_frame_maskRed2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Dictionary to match colours up to contours
        contours_dict = {
            "Yellow": [contoursYellow],
            "Blue": [contoursBlue],
            "Green": [contoursGreen],
            "Red": [contoursRed, contoursRed2]
        }
        
        # Add together all colour contours
        # Each colour is a list of contours
        # If colour is found remove contour list {contoursGreen = []}
        
        # When coloured thing is in center, reduce the turning of the robot
        # When something hit the scanners ranges[0], and coloured thing is in center then
        # report that the colour has been found and where it is 

        # Adds each colour that has not been found to contours list
        # contours = contoursYellow + contoursGreen + contoursBlue + contoursRed + contoursRed2
        contours = []
        for colour in self.colourFinding:
            if self.colourFinding[colour] == False:
                for col in contours_dict[colour]:
                    contours += col
        contoursYellow + contoursGreen + contoursBlue + contoursRed + contoursRed2
        

        # Sort by area (keep only the biggest one)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]

        # -----------
        # Sorts all the contours for checking what colour has been found
        contoursYellow = sorted(contoursYellow, key=cv2.contourArea, reverse=True)[:1]
        # -----------

        # Draw contour(s) (image to draw on, contours, contour number -1 to draw all contours, colour, thickness):
        current_frame_contours = cv2.drawContours(current_frame, contours, 0, (0, 255, 0), 20)
        
        if len(contours) > 0:
            if (len(contours[0]) > 10): # if largest contour is larger than x 
                # find the centre of the contour: https://docs.opencv.org/3.4/d8/d23/classcv_1_1Moments.html
                M = cv2.moments(contours[0]) # only select the largest controur


                if M['m00'] > 0:
                    # find the centroid of the contour
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    #print("Centroid of the biggest area: ({}, {})".format(cx, cy))



                    # Draw a circle centered at centroid coordinates
                    # cv2.circle(image, center_coordinates, radius, color, thickness) -1 px will fill the circle
                    cv2.circle(current_frame, (round(cx), round(cy)), 50, (0, 255, 0), -1)
                                
                    # find height/width of robot camera image from ros2 topic echo /camera/image_raw height: 1080 width: 1920

                    self.finding = True # Robot has an object in its sights

                    # if center of object is to the left of image center move left
                    if cx < 700: # 900
                        self.turn_vel = 0.3
                    # else if center of object is to the right of image center move right
                    elif cx >= 1000: #1200
                        self.turn_vel = -0.3
                    # Added more variations of turn speed to increace accuracy
                    elif cx < 900:
                        self.turn_vel = 0.1
                        self.speed = 0.1
                    elif cx >= 1200:
                        self.turn_vel = -0.1
                        self.speed = 0.1
                    # elif cx < 1000:
                    #     self.turn_vel = 0.05
                    #     self.speed = 0.05
                    # elif cx >= 1300:
                    #     self.turn_vel = -0.05
                    #     self.speed = 0.05
                    else: # center of object is in a x px range in the center of the image so dont turn
                        # print("Object in the center of image")
                        self.turn_vel = 0.0


                        # -----------
                        # Dealing with coloured objects
                        # if len(contoursYellow) > 0:
                        #     yellowMom = cv2.moments(contoursYellow[0])

                        #     yellowCx = int(yellowMom['m10']/yellowMom['m00'])
                        #     yellowCy = int(yellowMom['m01']/yellowMom['m00'])

                        #     if yellowCx == cx and yellowCy == cy:
                        #         print(f"Seen Yellow!")
                        #         self.possibleFinding["Yellow"] = True

                        # Make it possible to find all the colours
                        self.seen_colours("Yellow", contoursYellow, cx, cy)
                        self.seen_colours("Blue", contoursBlue, cx, cy)
                        self.seen_colours("Green", contoursGreen, cx, cy)
                        self.seen_colours("Red", contoursRed, cx, cy)
                        self.seen_colours("Red", contoursRed2, cx, cy)


                        # -----------
            else:
                # print("No Centroid Large Enougth Found")
                # turn until we can see a coloured object
                # self.turn_vel = 0.3
                self.turn_vel = 0.0 # Stop turning when none seen
                self.finding = False # Robot has no object in its sights
                        
        else:
            # print("No Centroid Found")
            # turn until we can see a coloured object
            # self.turn_vel = 0.3
            self.turn_vel = 0.0
            self.finding = False # Robot has no object in its sights

        # show the cv images
        current_frame_contours_small = cv2.resize(current_frame_contours, (0,0), fx=0.4, fy=0.4) # reduce image size
        cv2.imshow("Image window", current_frame_contours_small)
        cv2.waitKey(1)

    ## Method to check off colours that have been seen
    def seen_colours(self, colour, contour, cx, cy):
        if len(contour) > 0:
            if (len(contour[0]) > 10):
                moment = cv2.moments(contour[0])

                if moment['m00'] > 0:
                    contour_cx = int(moment['m10']/moment['m00'])
                    contour_cy = int(moment['m01']/moment['m00'])

                    if contour_cx == cx and contour_cy == cy:
                        print(f"Seen {colour}")
                        self.possibleFinding[colour] = True


    def timer_callback(self):
        self.tw = Twist() # twist message to publish

        self.tw.angular.z = self.turn_vel

        
        if (self.finding == True): # If robot has object in camera then turn towards it
            self.tw.linear.x = 0.25 # Move towards object
            self.pub_vel.publish(self.tw)

def main(args=None):
    print('Starting Colour Chaser.')

    rclpy.init(args=args)

    colour_chaser = ColourChaser()

    rclpy.spin(colour_chaser)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    colour_chaser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




## sudio apt update
## install it 
      # sudo apt install ros-hum - twiost mux
## move launch and config folder over
# Add path into teh data files
# in lauch change twist_mux/cmd_vel to /cmd_vel



# Change stuff in yaml files