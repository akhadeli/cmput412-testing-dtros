#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage

import cv2
from cv_bridge import CvBridge
import numpy as np

class Undistort(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(Undistort, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        # create window
        self._window = "Undistorted Image View"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        self._custom_topic = f"/{self._vehicle_name}/camera_node/undistorted_image/compressed"
        self._publisher = rospy.Publisher(self._custom_topic, CompressedImage)

    def callback(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)

        # Camera intrinsics
        K = np.array([[310.0149584021843, 0.0, 307.7177249518777],
                    [0.0, 309.29643750324607, 229.191787718834],
                    [0.0, 0.0, 1.0]], dtype=np.float32)

        D = np.array([-0.2683225140828933, 0.049595473114203516,
                    0.0003617920649662741, 0.006030049583437601, 0.0], dtype=np.float32)

        img_width, img_height = 640, 480

        # Compute optimal camera matrix to minimize black areas after undistortion
        new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (img_width, img_height), 1, (img_width, img_height))

        # Undistort image
        undistorted = cv2.undistort(image, K, D, None, new_K)

        # Crop the image based on ROI (Region of Interest)
        x, y, w, h = roi
        undistorted = undistorted[y:y+h, x:x+w]

        self._publisher.publish(self._bridge.cv2_to_compressed_imgmsg(undistorted))

        # display frame
        cv2.imshow(self._window, undistorted)

        cv2.waitKey(1)

if __name__ == '__main__':
    # create the node
    node = Undistort(node_name='undistort_node')
    # keep spinning
    rospy.spin()