#!/usr/bin/env python3

import os

import numpy as np
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage

import cv2
from cv_bridge import CvBridge

class LaneFollowing(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(LaneFollowing, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/undistorted_image/compressed"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        
        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

        self.proportional_gain = 0.00001

    def callback(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        h, w, _ = image.shape

        img_size = (w, h)
        
        src = np.float32([
            [0,382],
            [224, 191],  # Bottom left (near where left lane line is)
            [589, 382],  # Bottom right (near where right lane line is)
            [364, 191],  # Top left (near vanishing point for left lane)
        ])

        dst = np.float32([
            [100, 382],
            [100, -400],  # Bottom left (destination for left lane)
            [489, 382],  # Bottom right (destination for right lane)
            [489, -400],    # Top left (destination after warping)
        ])

        # cv2.circle(image, tuple(point), 5, (0, 0, 255), -1)  # Red dots

        M = cv2.getPerspectiveTransform(src, dst)
        
        warped = cv2.warpPerspective(image, M, img_size)

        # Convert warped image to HSV for color detection
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)

        # Define HSV range for detecting **white color**
        lower_white = np.array([0, 0, 200], dtype=np.uint8)
        upper_white = np.array([180, 50, 255], dtype=np.uint8)
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # Define HSV range for detecting **yellow color**
        lower_yellow = np.array([15, 100, 100], dtype=np.uint8)
        upper_yellow = np.array([35, 255, 255], dtype=np.uint8)
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Display all images in separate windows
        cv2.imshow("Before (Original Image)", image)  # Original image with source points
        cv2.imshow("After (Warped Image)", warped)    # Warped image
        cv2.imshow("White Lane Detection", mask_white)  # White mask
        cv2.imshow("Yellow Lane Detection", mask_yellow)  # Yellow mask

        # print(self.compute_error(mask_yellow=mask_yellow, target_x=100))
        bias = -569301
        print(self.compute_error(mask_yellow=mask_yellow, target_x=100) + self.compute_error(mask_yellow=mask_white, target_x=489) + bias)

        cv2.waitKey(1)
    
    def compute_error(self, mask_yellow, target_x=100):
        """
        Computes the sum of errors between detected yellow lane pixels and the expected lane position at x=100.
        
        Args:
            mask_yellow (numpy array): Binary image where white pixels represent detected yellow lane.
            target_x (int): Expected x-coordinate of the lane.

        Returns:
            float: The sum of errors between detected lane pixels and the expected position at x=100.
        """
        # Find nonzero (active) pixel coordinates
        y_coords, x_coords = np.where(mask_yellow > 0)  # y, x positions of active pixels
        
        if len(x_coords) == 0:
            return 0  # No detected yellow pixels, return 0 error

        # Compute the error as the difference from target_x
        errors = x_coords - target_x

        # Return the sum of the errors
        return np.sum(errors)

if __name__ == '__main__':
    # create the node
    node = LaneFollowing(node_name='lane_following')
    # keep spinning
    rospy.spin()