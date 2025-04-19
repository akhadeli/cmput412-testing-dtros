#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped, LEDPattern
import math
import time
from std_msgs.msg import ColorRGBA, Header
import types
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import numpy as np
from custom_utils.mask_operations import MaskOperations
from custom_utils.constants import Stall
from custom_utils.dtros_operations import DtrosOperations

class FinalBehaviorMainTask():
    def execute(self, dtros):
        self.onStart(dtros)
        self.runTask(dtros)
        self.onTearDown(dtros)

    def onStart(self, dtros):
        raise Exception("onStart() must be overriden")
    
    def onTearDown(self, dtros):
        DtrosOperations.unregister_and_delete_subscribers(dtros)
        
    def runTask(self, dtros):
        raise Exception("runTask() must be overriden")

class TailingTask(FinalBehaviorMainTask):
    pass

class AprilTagBehaviorTask(FinalBehaviorMainTask):
    pass

class ObstacleBehaviorTask(FinalBehaviorMainTask):
    pass

class ParkingTask(FinalBehaviorMainTask):
    pass

class FinalBehaviorMain(DTROS):
    def __init__(self, node_name, tasks):
        super(FinalBehaviorMain, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self._tasks = tasks

        self._vehicle_name = os.environ["VEHICLE_NAME"]
        self._wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        self._wheels_publisher = rospy.Publisher(self._wheels_topic, WheelsCmdStamped, queue_size=1)

    def run(self):
        for task in self._tasks:

            if not isinstance(task,FinalBehaviorMainTask):
                raise ValueError("task not recognized")

            print("Running " + str(task.__class__.__name__))

            task.execute(self)
            
        rospy.signal_shutdown(reason="tasks complete")

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._wheels_publisher.publish(stop)


class TurnRightTask(FinalBehaviorMainTask):
    def __init__(self, precision=40, tolerance=0.04, radians=math.pi/2, angular_velocity=1, R=0.4445):
        super().__init__()
        self._precision = precision
        self._tolerance = tolerance
        self._radians = radians
        self._angular_velocity=angular_velocity
        self._R = R

        self._ticks_left = None
        self._ticks_right = None

        self._distance_left = 0
        self._distance_right = 0

        self._resolution = 135
        self._l = 0.050

    def onStart(self, dtros):

        vehicle_name = os.environ["VEHICLE_NAME"]
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"

        self._radius = rospy.get_param(f'/{vehicle_name}/kinematics_node/radius', 0.0318)
        
        left_encoder_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        right_encoder_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"

        dtros._sub_left_wheel = rospy.Subscriber(left_encoder_topic, WheelEncoderStamped, self.callback_left_wheel)
        dtros._sub_right_wheel = rospy.Subscriber(right_encoder_topic, WheelEncoderStamped, self.callback_right_wheel)
        dtros._wheels_publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

    def callback_left_wheel(self, data):
        if self._ticks_left is None:
            self._ticks_left = data.data
            return
        
        self._distance_left += 2*math.pi*self._radius*((data.data - self._ticks_left)/self._resolution)
        self._ticks_left = data.data
    
    def callback_right_wheel(self, data):
        if self._ticks_right is None:
            self._ticks_right = data.data
            return
        
        self._distance_right += 2*math.pi*self._radius*((data.data - self._ticks_right)/self._resolution)
        self._ticks_right = data.data

    def runTask(self, dtros):
        
        precision = self._precision
        tolerance = self._tolerance
        target_radian = self._radians
        angular_velocity = self._angular_velocity
        R = self._R

        msg = f""" Running a curve task ... target_angle : {target_radian}, R: {R}, precision: {precision}, tolerance: {tolerance} """
        rospy.loginfo(msg)

        rate = rospy.Rate(precision)

        v_r = (R - self._l*3) * angular_velocity
        v_l = (R + self._l*3) * angular_velocity

        message = WheelsCmdStamped(vel_left=v_l, vel_right=v_r)

        while not rospy.is_shutdown():
            total_change_angle = ( self._distance_right - self._distance_left ) / (2*self._l)
            msg = f""" total_change_angle: {total_change_angle}, target_radian {target_radian}"""
            rospy.loginfo(msg)
            if abs(total_change_angle) >= (abs(target_radian) - tolerance):
                break

            dtros._wheels_publisher.publish(message)
            rate.sleep()

        msg = f""" total_change_angle: {total_change_angle}, target_radian {target_radian}"""
        rospy.loginfo(msg)

class TurnLeftTask(FinalBehaviorMainTask):
    def __init__(self, precision=40, tolerance=0.04, radians=math.pi/2, angular_velocity=1, R=0.4445):
        super().__init__()
        self._precision = precision
        self._tolerance = tolerance
        self._radians = radians
        self._angular_velocity=angular_velocity
        self._R = R

        self._ticks_left = None
        self._ticks_right = None

        self._distance_left = 0
        self._distance_right = 0

        self._resolution = 135
        self._l = 0.050

    def onStart(self, dtros):

        vehicle_name = os.environ["VEHICLE_NAME"]
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"

        self._radius = rospy.get_param(f'/{vehicle_name}/kinematics_node/radius', 0.0318)
        
        left_encoder_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        right_encoder_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"

        dtros._sub_left_wheel = rospy.Subscriber(left_encoder_topic, WheelEncoderStamped, self.callback_left_wheel)
        dtros._sub_right_wheel = rospy.Subscriber(right_encoder_topic, WheelEncoderStamped, self.callback_right_wheel)
        dtros._wheels_publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

    def callback_left_wheel(self, data):
        if self._ticks_left is None:
            self._ticks_left = data.data
            return
        
        self._distance_left += 2*math.pi*self._radius*((data.data - self._ticks_left)/self._resolution)
        self._ticks_left = data.data
    
    def callback_right_wheel(self, data):
        if self._ticks_right is None:
            self._ticks_right = data.data
            return
        
        self._distance_right += 2*math.pi*self._radius*((data.data - self._ticks_right)/self._resolution)
        self._ticks_right = data.data

    def runTask(self, dtros):
        
        precision = self._precision
        tolerance = self._tolerance
        target_radian = self._radians
        angular_velocity = self._angular_velocity
        R = self._R

        msg = f""" Running a curve task ... target_angle : {target_radian}, R: {R}, precision: {precision}, tolerance: {tolerance} """
        rospy.loginfo(msg)

        rate = rospy.Rate(precision)

        v_l = (R - self._l*3) * angular_velocity
        v_r = (R + self._l*3) * angular_velocity

        message = WheelsCmdStamped(vel_left=v_l, vel_right=v_r)

        while not rospy.is_shutdown():
            total_change_angle = ( self._distance_right - self._distance_left ) / (2*self._l)
            msg = f""" total_change_angle: {total_change_angle}, target_radian {target_radian}"""
            rospy.loginfo(msg)
            if abs(total_change_angle) >= (abs(target_radian) - tolerance):
                break

            dtros._wheels_publisher.publish(message)
            rate.sleep()

        msg = f""" total_change_angle: {total_change_angle}, target_radian {target_radian}"""
        rospy.loginfo(msg)

class PerpedicularAlignmentTask(FinalBehaviorMainTask):
    def __init__(self, stall, proportional_gain, derivative_gain, integral_gain, velocity, integral_saturation):
        
        if not isinstance(stall, Stall):
            raise Exception("stall must be of type Stall(Enum)")

        super().__init__()
        
        self._bridge = CvBridge()
        
        self.proportional_gain = proportional_gain
        self.derivate_gain = derivative_gain
        self.integral_gain = integral_gain
        self.vel = velocity
        self.integral_saturation = integral_saturation

        self._error = 20
        self._error_last = self._error
        self._integration_stored = 0

        self._yellow_lines = []
        self._white_lines = []

        self._homography_white_mask = None

        self._stall = stall

        self._curr_drive_direction = "Forward"

    def callback_homography_white_mask(self, msg):
        self._homography_white_mask = self._bridge.compressed_imgmsg_to_cv2(msg)

    def callback_undistorted_white_mask(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        self._white_mask = image
        undistorted_mask_white = image
        height = undistorted_mask_white.shape[0]
        undistorted_mask_white[:int(height / 4), :] = 0

        undistorted_mask_white = cv2.Canny(undistorted_mask_white, 50, 150)

        lines_yellow = cv2.HoughLines(undistorted_mask_white, 1, np.pi / 180, threshold=150)
        
        lines = []

        if lines_yellow is not None:
            lines.extend(lines_yellow[:, 0])  # flatten from shape (N, 1, 2) to (N, 2)
        
        self._white_lines = lines

        self.updateError()

        cv2.imshow("srad", undistorted_mask_white)
        cv2.waitKey(1)
        
    def callback_undistorted_yellow_mask(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        undistorted_mask_white = image
        height = undistorted_mask_white.shape[0]
        undistorted_mask_white[:int(height / 4), :] = 0

        undistorted_mask_white = cv2.Canny(undistorted_mask_white, 50, 150)

        lines_yellow = cv2.HoughLines(undistorted_mask_white, 1, np.pi / 180, threshold=100)
        
        lines = []

        if lines_yellow is not None:
            lines.extend(lines_yellow[:, 0])  # flatten from shape (N, 1, 2) to (N, 2)

        self._yellow_lines = lines

        self.updateError()

    def onStart(self, dtros):

        # construct subscriber
        self._undistorted_white_mask_topic = f"/{dtros._vehicle_name}/camera_node/undistorted_white_mask/compressed"
        self._sub_undistorted_white_mask = rospy.Subscriber(self._undistorted_white_mask_topic, CompressedImage, self.callback_undistorted_white_mask)

        # construct subscriber
        self._undistorted_yellow_mask_topic = f"/{dtros._vehicle_name}/camera_node/undistorted_yellow_mask/compressed"
        self._sub_undistorted_yellow_mask = rospy.Subscriber(self._undistorted_yellow_mask_topic, CompressedImage, self.callback_undistorted_yellow_mask)

        # construct subscriber
        self._homography_white_mask_topic = f"/{dtros._vehicle_name}/camera_node/homography_white_mask/compressed"
        self._sub_homography_white_mask = rospy.Subscriber(self._homography_white_mask_topic, CompressedImage, self.callback_homography_white_mask)
    
    def runTask(self, dtros):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            correctionUpdate = self.getUpdate()

            if self._curr_drive_direction == "Forward" and MaskOperations.getActiveCount(self._homography_white_mask) > 10000 and MaskOperations.getActiveCenter(self._homography_white_mask)[1] > 300:
                self._curr_drive_direction = "Backward"

            if self._curr_drive_direction == "Backward" and MaskOperations.getActiveCount(self._homography_white_mask) > 10000 and MaskOperations.getActiveCenter(self._homography_white_mask)[1] < 100:
                break

            if self._curr_drive_direction == "Forward":
                if correctionUpdate < 0:
                    message = WheelsCmdStamped(vel_left=self.vel, vel_right=self.vel+abs(correctionUpdate))
                elif correctionUpdate > 0:
                    message = WheelsCmdStamped(vel_left=self.vel+abs(correctionUpdate), vel_right=self.vel)
                else:
                    message = WheelsCmdStamped(vel_left=self.vel, vel_right=self.vel)
            else:
                if correctionUpdate < 0:
                    message = WheelsCmdStamped(vel_left=-1*self.vel-abs(correctionUpdate), vel_right=-1*self.vel)
                elif correctionUpdate > 0:
                    message = WheelsCmdStamped(vel_left=-1*self.vel, vel_right=-1*self.vel-abs(correctionUpdate))
                else:
                    message = WheelsCmdStamped(vel_left=-1*self.vel, vel_right=-1*self.vel)
            
            dtros._wheels_publisher.publish(message)
            
            rate.sleep()

    def updateError(self):
        lines = []
        lines.extend(self._yellow_lines) 
        lines.extend(self._white_lines)
        if lines:
            angles = []
            horiz_scores = []
            for rho, theta in lines:
                angle_deg = np.degrees(theta)
                horiz_score = min(abs(angle_deg), abs(angle_deg - 180))
                angles.append(angle_deg)
                horiz_scores.append(horiz_score)
            
            if len(angles) == 0 or horiz_scores == 0:
                return

            avg_angle = sum(angles) / len(angles)
            avg_score = 0 if 100 - abs(90 - avg_angle) <= 0 else 100 - abs(90 - avg_angle)

            print(f"Average Angle: {avg_angle:.2f}°, Average Score: {avg_score:.2f}")

            if avg_angle > 90:
                self._error = 100 - (avg_score)
            elif avg_score < 90:
                self._error = -1 * (100 - (avg_score))
            else:
                self._error = 0
        else:
            self._error = 0
            # print(self._error)

    def getUpdate(self):
        P = self._error*self.proportional_gain
        errorRateOfChange = self._error - self._error_last
        D = self.derivate_gain * errorRateOfChange
        integration_stored_update = self._integration_stored + (self._error)
        self._integration_stored = (integration_stored_update) if abs(integration_stored_update) <= self.integral_saturation else (integration_stored_update/integration_stored_update)*self.integral_saturation
        I = self.integral_gain * self._integration_stored

        self._error_last = self._error

        return P + I + D
        
if __name__ == "__main__":
    tasks = [
        # TurnLeftTask(R=0.7)
        PerpedicularAlignmentTask(stall=Stall.ONE, proportional_gain=0.05, derivative_gain=0.05, integral_gain=0, velocity=0.3, integral_saturation=100)
    ]
    node = FinalBehaviorMain(node_name="final_behavior_main_node", tasks=tasks)
    node.run()
    rospy.spin()