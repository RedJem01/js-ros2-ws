#!/usr/bin/env python

# An example of TurtleBot 3 subscribe to camera topic, mask colours, find and display contours, and move robot to center the object in image frame
# Written for humble
# cv2 image types - http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import math
import rclpy
from rclpy.node import Node
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, LaserScan, CameraInfo, PointCloud2, _point_cloud2
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np

class Assessment(Node):
    def __init__(self):
        super().__init__('colour_chaser')
        
        self.set_robot_current_pose = None
        self.set_robot_new_pose = None
        
        self.pointsXyz = []
        self.scan = None
        self.cameraInfo = None
        
        self.pushObject = False
        self.movedForwards = False
        
        self.leftRight = 0
        self.turnedToCubeLastCall = False

        #Publish cmd_vel topic to move the robot
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1)
        
        #Subscribe to scan topic
        self.create_subscription(LaserScan, '/scan', self.set_scan, 1)
        
        #Subscribe to the camera topic
        self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.camera_callback, 1)

        #Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def camera_callback(self, data):
        cv2.namedWindow("Image window", 1)

        #Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

        #Convert image to HSV
        current_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        #Create mask for range of colours (HSV low values, HSV high values)
        green_current_frame_mask = cv2.inRange(current_frame_hsv,(36, 0, 0), (70, 255, 255))

        green_contours, hierarchy = cv2.findContours(green_current_frame_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #Sort by area (keep only the biggest one)
        green_contours = sorted(green_contours, key=cv2.contourArea, reverse=True)

        #Draw contour(s) (image to draw on, contours, contour number -1 to draw all contours, colour, thickness):
        current_frame_contours = cv2.drawContours(current_frame, green_contours, 0, (255, 255, 0), 20)
        
        self.tw=Twist() #Twist message to publish
        
        #To stop robot getting confused between two cubes (if confused robot will turn left and right over and over again)
        print(self.leftRight)
        if self.leftRight >= 10:
            #Set movement so that robot can stop the loop
            self.tw.linear.x = 1.0
            self.leftRight = 0
        else:
            #If robot is currently in front of an object than pushObject = True
            #This part runs twice every time push object is set to true, once to move forwards and then another to move backwards so that the robot can see if it still has the cube with it
            if self.pushObject == True:
                self.push_object()
            else:
                if len(green_contours) > 0:
                    for c in green_contours:
                        # find the centre of the contour: https://docs.opencv.org/3.4/d8/d23/classcv_1_1Moments.html
                        M = cv2.moments(c) # only select the largest contour
                        if M['m00'] > 0:
                            # find the centroid of the contour
                            cx = int(M['m10']/M['m00'])
                            cy = int(M['m01']/M['m00'])
                            
                            #If contour is in the top half of the screen (is the wall)
                            if cy <= data.height / 2:
                                #If it is the only contour then turn, if not then go to next contour
                                if len(green_contours) == 1:
                                    print("Wall is only contour, turning")
                                    self.tw.angular.z = 1.0
                                continue
                            
                            # Draw a circle centered at centroid coordinates
                            cv2.circle(current_frame, (round(cx), round(cy)), 5, (0, 255, 0), -1)

                            #If center of object is to the left of image center move left
                            if cx < 2 * data.width / 5:
                                print("turning left")
                                self.tw.angular.z = 0.3
                                self.stop_looping()
                            #Else if center of object is to the right of image center move right
                            elif cx >= 3 * data.width / 5:
                                print("turning right")
                                self.tw.angular.z = -0.3
                                self.stop_looping()
                            else: 
                                self.turnedToCubeLastCall = False
                                #If robot not at wall
                                if self.scan.ranges[90] > 0.6:
                                    #Get the depth of the cube
                                    cubeDepth = (self.get_depth(current_frame, c) / 100)
                                    #If the difference between the wall and the cube is less than 0.03
                                    if (abs(self.scan.ranges[90] - cubeDepth)) < 0.02:
                                        #Big turn so we don't keep getting stuck on the same cube again
                                        self.tw.angular.z = 3.0
                                        break
                                    
                                    #Move towards object
                                    print("Moving towards object")
                                    self.tw.angular.z = 0.0
                                    self.tw.linear.x = 0.5
                                    
                                    #Check if cube is at the very bottom of the screen and if it is then the robot is pushing the object
                                    if cy >= (3 * data.height / 4):
                                        self.pushObject = True
                                        break
                                else:
                                    #If at the wall then big turn to see other cubes
                                    print("At wall, turning")
                                    self.tw.angular.z=3.0
                            break                   
                else:
                    self.turnedToCubeLastCall = False
                    #Turn until we can see a coloured object
                    self.tw.angular.z=1.0

        #Publish the twist message to the robot
        print(self.tw)
        self.pub_cmd_vel.publish(self.tw)

        #Show the cv images
        current_frame_contours_small = cv2.resize(current_frame_contours, (0,0), fx=0.4, fy=0.4) # reduce image size
        cv2.imshow("Image window", current_frame_contours_small)
        cv2.waitKey(1)
        
    def push_object(self):
        #Robot is not turning to a cube this call
        self.turnedToCubeLastCall = False
        #If push object is true and the robot has not moved forwards yet
        if self.movedForwards == False:
            #Move forwards
            self.tw.linear.x = 1.0
            self.movedForwards = True
        #If has moved forwards
        else:
            #Move backwards
            self.tw.linear.x = -2.0
            self.movedForwards = False
            self.pushObject = False    
        
    #Adapted from https://pyimagesearch.com/2015/05/25/basic-motion-detection-and-tracking-with-python-and-opencv/
    def get_depth(self, current_frame, c):
        #Get depth by drawing box around the contour then getting the mean of the pixel at the top right of the box
        x, y, w, h = cv2.boundingRect(c)
        objectDepth = current_frame[y:y+h, x:x+w]
        return np.mean(objectDepth)
    
    def stop_looping(self):
        #To stop robot getting confused between two cubes
        if self.turnedToCubeLastCall == True:
            self.leftRight += 1
        else:
            self.leftRight = 1
        self.turnedToCubeLastCall = True

    def set_scan(self, laserScan):
        self.scan = laserScan

def main(args=None):
    print('Starting Assessment.py.')

    rclpy.init(args=args)

    assessment = Assessment()

    rclpy.spin(assessment)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    assessment.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()