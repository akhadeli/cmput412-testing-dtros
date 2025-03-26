#!/usr/bin/env python3
import dt_apriltags
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
from enum import Enum
import numpy as np
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import String
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA
from apriltags import AprilTag

class Colors(Enum):
    Red = [1.0, 0.0, 0.0]
    Green = [0.0, 1.0, 0.0]
    Blue = [0.0, 0.0, 1.0]
    Yellow = [1.0, 1.0, 0.0]
    Teal = [0.0, 1.0, 1.0]
    Magenta = [1.0, 0.0, 1.0]
    Off = [0.0, 0.0, 0.0]
    DarkOrange = [1.0, 0.55, 0]

class AprilTagDetection(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(AprilTagDetection, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        # bridge between OpenCV and ROS
        self._bridge = CvBridge()

        self._state_topic = f"/{self._vehicle_name}/state"
        self._state_publisher = rospy.Publisher(self._state_topic, String, queue_size=1)

        self.undistort_gray_topic = f"/{self._vehicle_name}/camera_node/undistort_gray/compressed"
        self.undistort_gray_sub = rospy.Subscriber(self.undistort_gray_topic, CompressedImage, self.undistorted_gray_callback)

        # Initialize AprilTag detector **only once**
        self.detector = dt_apriltags.Detector(families="tag36h11")

        self.detected_state = None

    def undistorted_gray_callback(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)

        # Convert to grayscale
        # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = image

        # Detect AprilTags
        results = self.detector.detect(gray)

        self.detected_state = "No tag detected"
        # Loop through detected tags and print tag IDs
        for result in results:
            if result.tag_id == AprilTag.UALBERTA.value:
                self.detected_state = "UALBERTA tag detected"
            elif result.tag_id == AprilTag.INTERSECTIONT.value:
                self.detected_state =  "INTERSECTIONT tag detected"
            elif result.tag_id == AprilTag.STOP.value:
                self.detected_state = "STOP tag detected"
            break
        print(len(results))
        print(self.detected_state)
        self._state_publisher.publish(self.detected_state)


        


if __name__ == '__main__':
    # create the node
    node = AprilTagDetection(node_name='april_tag_detection_node')
    # node.run()
    # keep spinning
    rospy.spin()