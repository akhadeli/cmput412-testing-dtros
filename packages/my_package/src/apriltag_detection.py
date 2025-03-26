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

class Colors(Enum):
    Red = [1.0, 0.0, 0.0]
    Green = [0.0, 1.0, 0.0]
    Blue = [0.0, 0.0, 1.0]
    Yellow = [1.0, 1.0, 0.0]
    Teal = [0.0, 1.0, 1.0]
    Magenta = [1.0, 0.0, 1.0]
    Off = [0.0, 0.0, 0.0]
    DarkOrange = [1.0, 0.55, 0]

class State():
    def __init__(self, message_name, colorPattern):
        if(not isinstance(colorPattern, ColorPattern)):
            raise Exception("colorPattern must be of type ColorPattern")
        self.message_name = message_name
        self.led_colors = colorPattern.getColorMask()
    
    def getLedMessage(self):
        led_msg = LEDPattern()

        for color in self.led_colors:
            # Color for the LEDs
            rgba = ColorRGBA()
            rgba.r = color[0]
            rgba.g = color[1]
            rgba.b = color[2]
            rgba.a = 1.0

            led_msg.rgb_vals.append(rgba)
        
        return led_msg

class ColorPattern():
    def __init__(self, frontLeft, frontRight, backLeft, backRight):
        if (not isinstance(frontLeft, Colors) or 
            not isinstance(frontRight, Colors) or
            not isinstance(backLeft, Colors) or 
            not isinstance(backRight, Colors)):
            raise Exception("Parameters of ColorPattern must be of type Colors(Enum)")
        self.frontLeft = frontLeft
        self.frontRight = frontRight
        self.backLeft = backLeft
        self.backRight = backRight

    def getColorMask(self):
        return [self.frontLeft.value, self.backRight.value, [0,0,0],  self.backLeft.value, self.frontRight.value]
    


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            for state in self.states:
                if self._current_state is not None and state.message_name == self._current_state:
                    self._led_publisher.publish(state.getLedMessage())
                    break
            rate.sleep()

class AprilTag(Enum):
    UALBERTA = 201
    INTERSECTIONT = 133
    STOP = 21

class AprilTagDetection(DTROS):

    def __init__(self, node_name, proportional_gain=0.0000002, derivative_gain=0.0000002, integral_gain=0.0000002, velocity=0.3, integral_saturation=500000):
        # initialize the DTROS parent class
        super(AprilTagDetection, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        self._led_publisher = rospy.Publisher(f'/{self._vehicle_name}/led_emitter_node/led_pattern', LEDPattern, queue_size=10)
        self.states = [
            State(message_name="No tag detected", colorPattern=ColorPattern(frontLeft=Colors.Off, frontRight=Colors.Off, backLeft=Colors.Off, backRight=Colors.Off)),
            State(message_name="INTERSECTIONT tag detected", colorPattern=ColorPattern(frontLeft=Colors.Blue, frontRight=Colors.Blue, backLeft=Colors.Blue, backRight=Colors.Blue)),
            State(message_name="STOP tag detected", colorPattern=ColorPattern(frontLeft=Colors.Red, frontRight=Colors.Red, backLeft=Colors.Red, backRight=Colors.Red)),
            State(message_name="UALBERTA tag detected", colorPattern=ColorPattern(frontLeft=Colors.Green, frontRight=Colors.Green, backLeft=Colors.Green, backRight=Colors.Green)),
        ]

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

        self._state_topic = f"/{self._vehicle_name}/state"
        self._state_publisher = rospy.Publisher(self._state_topic, String, queue_size=1)

        self.undistort_gray_topic = f"/{self._vehicle_name}/camera_node/undistort_gray/compressed"
        self.undistort_gray_sub = rospy.Subscriber(self.undistort_gray_topic, CompressedImage, self.undistorted_gray_callback)

        self._camera_topic = f"/{self._vehicle_name}/camera_node/undistorted_image/compressed"
        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.undistorted_callback)

        # Initialize AprilTag detector **only once**
        self.detector = dt_apriltags.Detector(families="tag36h11")

        self.detected_state = None

    def undistorted_gray_callback(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)

        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

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

    def undistorted_callback(self, msg):
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

        self._error = self.compute_error(mask=mask_yellow, target_x=100, pixel_value=1) + self.compute_error(mask=mask_white, target_x=489)

        # # Display all images in separate windows
        # cv2.imshow("Before (Original Image)", image)  # Original image with source points
        # cv2.imshow("After (Warped Image)", warped)    # Warped image
        # # cv2.imshow("White Lane Detection", mask_white)  # White mask
        # # cv2.imshow("Yellow Lane Detection", mask_yellow)  # Yellow mask
        # cv2.waitKey(1)
    
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            correctionUpdate = self.getUpdate()
            print(self.detected_state)
            for state in self.states:
                if self.detected_state is not None and state.message_name == self.detected_state:
                    self._led_publisher.publish(state.getLedMessage())
                    break
        
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
    node = AprilTagDetection(node_name='april_tag_detection_node')
    node.run()
    # keep spinning
    rospy.spin()