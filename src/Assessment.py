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

class ColourChaser(Node):
    def __init__(self):
        super().__init__('colour_chaser')
        
        self.set_robot_current_pose = None
        self.set_robot_new_pose = None
        
        self.pointsXyz = []
        self.scan = None
        self.cameraInfo = None
        
        self.pushingObject = False
        
        # self.foundCubes = [] 
        self.doneCubes = [] #In form [position list] e.g. [[0, 0 ,0]]
        
        self.cubeId = None
        self.position = [] #In form [x,y,z]
        
        # self.cubeExists = False
        self.cubeDone = False

        self.addCube = False

        #Publish cmd_vel topic to move the robot
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1)
        
        #Subscribe to scan topic
        self.create_subscription(LaserScan, '/scan', self.set_scan, 1)

        #Subscribe to camera info topic
        self.create_subscription(CameraInfo, '/limo/depth_camera_link/camera_info', self.set_camera_info, 1)
        
        #Subscribe to the camera points topic
        self.create_subscription(PointCloud2, '/limo/depth_camera_link/points', self.set_points, 1)
        
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
        current_frame_mask = cv2.inRange(current_frame_hsv,(36, 0, 0), (70, 255,255))

        contours, hierarchy = cv2.findContours(current_frame_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #Sort by area (keep only the biggest one)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        #Draw contour(s) (image to draw on, contours, contour number -1 to draw all contours, colour, thickness):
        current_frame_contours = cv2.drawContours(current_frame, contours, 0, (255, 255, 0), 20)
        
        self.tw=Twist() # twist message to publish
        
        # self.cubeExists = False
        self.cubeDone = False
        
        #If robot is currently in front of an object than pushingObject = True
        if self.pushingObject == True:
            #If robot is at the wall
            if self.scan.ranges[90] < 0.3:
                #Move backwards so that the robot can get the position of the done cube
                self.tw.linear.x = -1.0
                self.pushingObject = False
                self.addCube = True
            else:
                #Move object forwards
                self.tw.linear.x = 0.5
        else:
            if len(contours) > 0:
                for c in contours:
                    # find the centre of the contour: https://docs.opencv.org/3.4/d8/d23/classcv_1_1Moments.html
                    M = cv2.moments(c) # only select the largest contour
                    if M['m00'] > 0:
                        # find the centroid of the contour
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        
                        print("Centroid of the biggest area: ({}, {})".format(cx, cy))
                        
                        #If contour is in the top half of the screen (is the wall)
                        if cy <= data.height / 2:
                            #If it is the only contour then turn, if not then go to next contour
                            if len(contours) == 1:
                                self.tw.angular.z = 0.6
                            continue
                        
                        # Draw a circle centered at centroid coordinates
                        # cv2.circle(image, center_coordinates, radius, color, thickness) -1 px will fill the circle
                        cv2.circle(current_frame, (round(cx), round(cy)), 5, (0, 255, 0), -1)
                        
                        #Get the position of the cube
                        self.get_position(current_frame, c)
                        
                        print("ac", self.addCube)
                        #If the robot has just finished pushing the cube to the wall (put here since it needs the new position)
                        if self.addCube == True:
                            self.doneCubes.append(self.position)
                        
                        #Check cubes
                        self.cube_check()
                        
                        #If cube is already at wall then go to next contour
                        print("cd", self.cubeDone)
                        if self.cubeDone == True:
                            print("ci", contours.index(c))
                            print("cl", (len(contours) + 1))
                            if contours.index(c) < (len(contours) + 1):
                                continue
                            else:
                                self.tw.angular.z=0.6
                        
                        # find height/width of robot camera image from ros2 topic echo /camera/image_raw height: 1080 width: 1920
                        #If center of object is to the left of image center move left
                        if cx < 2 * data.width / 5:
                            self.tw.angular.z = 0.3
                        #Else if center of object is to the right of image center move right
                        elif cx >= 4 * data.width / 5:
                            self.tw.angular.z = -0.3
                        else: 
                            #If not at wall
                            if self.scan.ranges[90] > 0.3:
                                #If cube is not done
                                    #Move
                                    self.tw.angular.z = 0.0
                                    self.tw.linear.x = 0.5
                                    #Check if cube is at the very bottom of the screen
                                    if cy >= (3 * data.height / 4):
                                        self.pushingObject = True
                            else:
                                #Turn
                                self.tw.angular.z=0.6
                        break                   
            else:
                #Make sure that if the cube hasn't moved with the robot, it won't declare the next cube it sees as done
                self.delCube = False
                # turn until we can see a coloured object
                self.tw.angular.z=0.6

        self.pub_cmd_vel.publish(self.tw)
        print(self.doneCubes)

        # show the cv images
        current_frame_contours_small = cv2.resize(current_frame_contours, (0,0), fx=0.4, fy=0.4) # reduce image size
        cv2.imshow("Image window", current_frame_contours_small)
        cv2.waitKey(1)
        
    def set_points(self, points):
        xyz_array = _point_cloud2.read_points(points, skip_nans=True, field_names=("x", "y", "z"))
        
    def cube_check(self):
        #Check if the cube in the contour is already done
        for cu in self.doneCubes:
            if (cu[1] == self.position):
                self.cubeDone = True
        
    def get_position(self, current_frame, c):
        #Get coords
        contour_pixels = c.reshape(-1, 2)
        for pixel in contour_pixels:
            u, v = pixel[0], pixel[1]
        
        #Get depth
        x, y, w, h = cv2.boundingRect(c)
        objectDepth = current_frame[y:y+h, x:x+w]
        avgDepth = np.mean(objectDepth)
        
        # https://answers.ros.org/question/195737/how-to-get-coordinates-from-depthimage/
        #Get position
        cam_model = PinholeCameraModel()
        cam_model.fromCameraInfo(self.cameraInfo)
        ray = np.array(cam_model.projectPixelTo3dRay((u,v)))
        self.position = ray * avgDepth
        self.position = [math.floor(self.position[0]), math.floor(self.position[1]), math.floor(self.position[2])]


    def set_scan(self, laserScan):
        self.scan = laserScan
        
    def set_camera_info(self, cameraInfo):
        self.cameraInfo = cameraInfo

def main(args=None):
    print('Starting colour_chaser.py.')

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