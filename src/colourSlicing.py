#!/usr/bin/env python

# Written for humble
# cv2 image types - http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np

class ColourContours(Node):
    def __init__(self):
        super().__init__('colour_contours')
        self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.camera_callback, 1)
        self.create_publisher(String, '/msgs', 1)

        self.br = CvBridge()

    def camera_callback(self, data):
        #self.get_logger().info("camera_callback")

        try:
            cv_image = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        bright = np.uint8([[[255,0,0 ]]])
        hsv_bright = cv2.cvtColor(bright, cv2.COLOR_BGR2HSV)
    
        # preparing the mask to overlay 
        mask = cv2.inRange(hsv_img, np.array((0, 150, 50)),
                                 np.array((255, 255, 255))) 
        
        # The black region in the mask has the value of 0, 
        # so when multiplied with original image removes all non-blue regions 
        result = cv2.bitwise_and(cv_image, cv_image, mask = mask) 

        cv_image_small = cv2.resize(cv_image, (0,0), fx=0.4, fy=0.4) # reduce image size
        mask_small = cv2.resize(mask, (0,0), fx=0.4, fy=0.4)
        result_small = cv2.resize(result, (0,0), fx=0.4, fy=0.4)
        
        print(np.mean(cv_image_small))
        cv2.imshow('frame', cv_image_small)
        cv2.imshow('mask', mask_small)
        cv2.imshow('result', result_small)
        cv2.waitKey(1)

def main(args=None):
    print('Starting colour_contours.py.')

    rclpy.init(args=args)

    colour_contours = ColourContours()

    rclpy.spin(colour_contours)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    colour_contours.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()