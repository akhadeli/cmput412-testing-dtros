#!/usr/bin/env python3

import os
import numpy as np
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
import cv2
from cv_bridge import CvBridge
import time

class RightDrivingVehicleAvoidance(DTROS):

    def __init__(self, node_name, proportional_gain=0.0000002, derivative_gain=0.0000002, integral_gain=0.0000002, velocity=0.3, integral_saturation=500000, duckie_detection_sensitivity=2000, duckie_detection_distance=50000, lane_correction_delay=2):
        # initialize the DTROS parent class
        super(RightDrivingVehicleAvoidance, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/undistorted_image/compressed"
        wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        
        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

        self.proportional_gain = proportional_gain
        self.derivate_gain = derivative_gain
        self.integral_gain = integral_gain
        self.vel = velocity
        self.integral_saturation = integral_saturation

        self._error = 20
        self._error_last = self._error
        self._integration_stored = 0

        self._blue_detection_topic = f"/{self._vehicle_name}/camera_node/blue_image_mask/compressed"
        self.blue_mask_sub = rospy.Subscriber(self._blue_detection_topic , CompressedImage, self.blue_detection_callback)

        self._duckie_detected = False
        self._duckie_detected_time_stamp = None

        self.duckie_detection_sensitivity = duckie_detection_sensitivity
        self.duckie_detection_distance = duckie_detection_distance

        self.lane_correction_delay = lane_correction_delay
    
    def blue_detection_callback(self, msg):
        # msg is already a mask
        mask_blue = self._bridge.compressed_imgmsg_to_cv2(msg)

        # Find the coordinates of active (non-zero) pixels
        y_coords, x_coords = np.where(mask_blue > 0)  # y, x positions of active pixels

        if len(x_coords) == 0:
            # print("Detecting nothing")
            return

        # Calculate the variance of the x coordinates
        x_variance = np.var(x_coords)

        # Calculate the magnitude (number of active blue pixels)
        magnitude = len(x_coords)

        # print("Magnitude : " + str(magnitude) + ", Variance : " + str(x_variance))

        if (magnitude >= self.duckie_detection_sensitivity):
            if(x_variance <= self.duckie_detection_distance):
                self._duckie_detected = True
                self._duckie_detected_time_stamp = time.time()
                return
        
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
            [100, 0],  # Bottom left (destination for left lane)
            [489, 382],  # Bottom right (destination for right lane)
            [489, 0],    # Top left (destination after warping)
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

        if self._duckie_detected and self._duckie_detected_time_stamp != None and (time.time() - self._duckie_detected_time_stamp) < self.lane_correction_delay:
            print("Target : left lane")
            yellow_error = self.compute_error(mask=mask_yellow, target_x=489)
            white_error = self.compute_error(mask=mask_white, target_x=100)
        else:
            print("Target : right lane")
            yellow_error = self.compute_error(mask=mask_yellow, target_x=100)
            white_error = self.compute_error(mask=mask_white, target_x=489)
            
        if yellow_error > 100000 and white_error < -100000:
            self._error = (yellow_error + abs(white_error)) 
        elif yellow_error < -100000 and white_error > 100000:
            self._error = (yellow_error + -1*white_error) 
        else:
            self._error = yellow_error + white_error

        # self._error = yellow_error + white_error

    
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
    
    def compute_error(self, mask, target_x=100, pixel_value=1):
        """
        Computes the sum of errors between detected yellow lane pixels and the expected lane position at x=100.
        
        Args:
            mask_yellow (numpy array): Binary image where white pixels represent detected yellow lane.
            target_x (int): Expected x-coordinate of the lane.

        Returns:
            float: The sum of errors between detected lane pixels and the expected position at x=100.
        """
        # Find nonzero (active) pixel coordinates
        y_coords, x_coords = np.where(mask > 0)  # y, x positions of active pixels
        
        if len(x_coords) == 0:
            return 0  # No detected yellow pixels, return 0 error

        # Compute the error as the difference from target_x
        errors = (x_coords - target_x) * pixel_value

        # Return the sum of the errors
        return np.sum(errors)
    
    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)

if __name__ == '__main__':
    # create the node
    node = RightDrivingVehicleAvoidance(node_name="PID_controller_node", proportional_gain=0.0000002, derivative_gain=0.0000002, integral_gain=0.0000002, velocity=0.3, integral_saturation=500000, duckie_detection_sensitivity=2000, duckie_detection_distance=30000, lane_correction_delay=2)
    node.run()
    # keep spinning
    rospy.spin()