#!/usr/bin/env python3

import os
import numpy as np
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
import cv2
from cv_bridge import CvBridge

class ColorDetection(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ColorDetection, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']

        self._red_detection_topic = f"/{self._vehicle_name}/camera_node/red_image_mask/compressed"
        self._red_publisher = rospy.Publisher(self._red_detection_topic, CompressedImage)

        self._blue_detection_topic = f"/{self._vehicle_name}/camera_node/blue_image_mask/compressed"
        self._blue_publisher = rospy.Publisher(self._blue_detection_topic, CompressedImage)

        self._green_detection_topic = f"/{self._vehicle_name}/camera_node/green_image_mask/compressed"
        self._green_publisher = rospy.Publisher(self._green_detection_topic, CompressedImage)

        self._green_detection_topic = f"/{self._vehicle_name}/camera_node/green_image_mask/compressed"
        self._green_publisher = rospy.Publisher(self._green_detection_topic, CompressedImage)

        self._undistorted_topic = f"/{self._vehicle_name}/camera_node/undistorted_image/compressed"
        self.sub = rospy.Subscriber(self._undistorted_topic, CompressedImage, self.callback)

        self._bridge = CvBridge()

    def callback(self, msg):
        # Convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)

        # Convert warped image to HSV for color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0, 100, 100], dtype=np.uint8)  # Lower bound for red
        upper_red = np.array([10, 255, 255], dtype=np.uint8)  # Upper bound for red
        mask_red = cv2.inRange(hsv, lower_red, upper_red)

        lower_blue = np.array([100, 150, 0], dtype=np.uint8)  # Lower bound for blue
        upper_blue = np.array([140, 255, 255], dtype=np.uint8)  # Upper bound for blue
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        lower_green = np.array([40, 40, 150], dtype=np.uint8)  # Lower bound for green (close to #56B49C)
        upper_green = np.array([90, 255, 255], dtype=np.uint8)  # Upper bound for green (close to #56B49C)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        # Publish the masks
        self._blue_publisher.publish(self._bridge.cv2_to_compressed_imgmsg(mask_blue))
        self._red_publisher.publish(self._bridge.cv2_to_compressed_imgmsg(mask_red))
        self._green_publisher.publish(self._bridge.cv2_to_compressed_imgmsg(mask_green))

if __name__ == '__main__':
    # create the node
    node = ColorDetection(node_name="color_detection_node")
    # keep spinning
    rospy.spin()