#!/usr/bin/env python3

import os
import numpy as np
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
import cv2
from cv_bridge import CvBridge
import dt_apriltags

class PIDController(DTROS):

    def __init__(self, node_name, proportional_gain=0.0000002, derivative_gain=0.0000002, integral_gain=0.0000002, velocity=0.3, integral_saturation=500000):
        # initialize the DTROS parent class
        super(PIDController, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        
        self.proportional_gain = proportional_gain
        self.derivate_gain = derivative_gain
        self.integral_gain = integral_gain
        self.vel = velocity
        self.integral_saturation = integral_saturation

        self._error = 20
        self._error_last = self._error
        self._integration_stored = 0

        # Initialize AprilTag detector **only once**
        self.detector = dt_apriltags.Detector(families="tag36h11")

        self.detected_state = None

        self.apriltag_center_offset_error = 0

        self.yellow_line_verticality_error = 0

        # construct subscriber
        self._homography_yellow_mask_topic = f"/{self._vehicle_name}/camera_node/homography_yellow_mask/compressed"
        self.homography_yellow_mask_sub = rospy.Subscriber(self._homography_yellow_mask_topic, CompressedImage, self.callback_homography_yellow_mask)

        # construct subscriber
        self._undistorted_gray_topic = f"/{self._vehicle_name}/camera_node/undistort_gray/compressed"
        self.undistorted_gray_sub = rospy.Subscriber(self._undistorted_gray_topic, CompressedImage, self.callback_undistorted_gray)

    def callback_undistorted_gray(self, msg):
        image = self._bridge.compressed_imgmsg_to_cv2(msg)

        #  # If the image is grayscale (single channel), convert it to BGR
        # if len(image.shape) == 2:
        #     image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        # # Detect AprilTags (assuming detector works on grayscale, you might need to convert back or detect on the original)
        # results = self.detector.detect(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        
        # for r in results:
        #     if r.tag_id == 47:
        #         # Compute center of image
        #         image_center_x = image.shape[1] // 2  # image width divided by 2

        #         # Compute x-distance from image center to AprilTag center
        #         dx = r.center[0] - image_center_x  # tag center x - image center x

        #         self.apriltag_center_offset_error = dx

        #         self._error = self.apriltag_center_offset_error + self.yellow_line_verticality_error
        #         print(f"AprilTag ID {r.tag_id} is {dx:.1f} pixels from image center along x-axis")

    def callback_homography_yellow_mask(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        # Optionally apply edge detection
        edges = cv2.Canny(image, 50, 150)
        lines = cv2.HoughLines(edges, 1, np.pi / 180, threshold=150)
        if lines is not None:
            angles = []
            scores = []
            for rho, theta in lines[:, 0]:
                angle_deg = np.degrees(theta)
                angle_from_vertical = min(abs(angle_deg), abs(180 - angle_deg))
                verticality_score = 100 * np.cos(np.radians(angle_from_vertical))

                # # Determine sign: angles < 90 → leaning left (negative), > 90 → leaning right (positive)
                # if angle_deg < 90 or angle_deg > 180:
                #     signed_angle = -angle_from_vertical  # leaning left
                # else:
                #     signed_angle = angle_from_vertical   # leaning right

                angles.append(angle_deg)
                scores.append(verticality_score)

                # Draw the line
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * a)
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * a)
                cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                
            if len(angles) == 0 or len(scores) == 0:
                return
            avg_angle = sum(angles) / len(angles)
            avg_score = sum(scores) / len(scores)

            if (avg_angle < 90) :
                self.yellow_line_verticality_error = (100 - avg_score)
            elif (avg_angle > 90):
                self.yellow_line_verticality_error = -1*(100 - avg_score)
            else:
                self.yellow_line_verticality_error = 0

            self._error = self.apriltag_center_offset_error + self.yellow_line_verticality_error

            # print(f"Avg Angle: {avg_angle:.2f}°, Avg Verticality Score: {avg_score:.3f}")

    
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            correctionUpdate = self.getUpdate()
            
            if correctionUpdate < 0:
                message = WheelsCmdStamped(vel_left=self.vel, vel_right=self.vel+abs(correctionUpdate))
            elif correctionUpdate > 0:
                message = WheelsCmdStamped(vel_left=self.vel+abs(correctionUpdate), vel_right=self.vel)
            else:
                message = WheelsCmdStamped(vel_left=self.vel, vel_right=self.vel)
            
            self._publisher.publish(message)
            
            rate.sleep()

    def getUpdate(self):
        P = self._error*self.proportional_gain
        errorRateOfChange = self._error - self._error_last
        D = self.derivate_gain * errorRateOfChange
        integration_stored_update = self._integration_stored + (self._error)
        self._integration_stored = (integration_stored_update) if abs(integration_stored_update) <= self.integral_saturation else (integration_stored_update/integration_stored_update)*self.integral_saturation
        I = self.integral_gain * self._integration_stored

        self._error_last = self._error

        return P + I + D
    
    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)

if __name__ == '__main__':
    # create the node
    node = PIDController(node_name="PID_controller_node", proportional_gain=0.01, derivative_gain=0.01, integral_gain=0, velocity=0.1, integral_saturation=100)
    node.run()
    # keep spinning
    rospy.spin()