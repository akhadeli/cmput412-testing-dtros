from custom_utils.dtros_operations import DtrosOperations
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped, LEDPattern
import math
import time
from std_msgs.msg import ColorRGBA, Header
import types
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from custom_utils.mask_operations import MaskOperations
from custom_utils.pid_operations import PIDOperations
from custom_utils.image_operations import ImageOperations
import dt_apriltags
import numpy as np
import rospy
import os
from custom_utils.constants import Stall

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
    
class TurnRightTask(FinalBehaviorMainTask):
    def __init__(self, precision=40, tolerance=0, radians=math.pi/2, angular_velocity=2.1, R=0.3):
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

        v_r = (R + self._l*3) * angular_velocity
        v_l = (R - self._l*3) * angular_velocity

        message = WheelsCmdStamped(vel_left=v_l, vel_right=v_r)

        while not rospy.is_shutdown():
            total_change_angle = ( self._distance_right - self._distance_left ) / (2*self._l)
            msg = f""" total_change_angle: {total_change_angle}, target_radian {target_radian}"""
            rospy.loginfo(msg)
            if abs(total_change_angle) >= (abs(target_radian) - tolerance):
                message = WheelsCmdStamped(vel_left=0, vel_right=0)
                dtros._wheels_publisher.publish(message)
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
                message = WheelsCmdStamped(vel_left=0, vel_right=0)
                dtros._wheels_publisher.publish(message)
                break

            dtros._wheels_publisher.publish(message)
            rate.sleep()

        msg = f""" total_change_angle: {total_change_angle}, target_radian {target_radian}"""
        rospy.loginfo(msg)

class StallAlignmentTask(FinalBehaviorMainTask):
    def __init__(self, target_stall, proportional_gain, derivative_gain, integral_gain, velocity, integral_saturation):
        
        if not isinstance(target_stall, Stall):
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

        self._target_stall = target_stall

        self._curr_drive_direction = "Forward"

    def callback_raw_image(self, msg):
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        undistorted = ImageOperations.undistort(image)
        homography = ImageOperations.getHomography(undistorted)
        self._homography_white_mask = ImageOperations.getWhiteMask(homography)
        yellow_mask = ImageOperations.getYellowMask(homography)
        white_mask = self._homography_white_mask

        lines_white = ImageOperations.getMaskLines(white_mask)

        lines_yellow = ImageOperations.getMaskLines(yellow_mask)

        self._white_lines = lines_white
        self._yellow_lines = lines_yellow

        self.updateError()

        cv2.imshow("Undistort White Mask", image)
        cv2.waitKey(1)

    def onStart(self, dtros):

        # construct subscriber
        self._raw_image_topic = f"/{dtros._vehicle_name}/camera_node/image/compressed"
        dtros._sub_raw_image = rospy.Subscriber(self._raw_image_topic, CompressedImage, self.callback_raw_image)

    def runTask(self, dtros):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            correctionUpdate = self.getUpdate()

            if self._curr_drive_direction == "Forward" and MaskOperations.getActiveCount(self._homography_white_mask) > 10000 and MaskOperations.getActiveCenter(self._homography_white_mask)[1] > 300:
                if self._target_stall in [Stall.TWO, Stall.FOUR]:
                    break
                self._curr_drive_direction = "Backward"

            if self._curr_drive_direction == "Backward" and MaskOperations.getActiveCenter(self._homography_white_mask)[1] < 150:
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

        message = WheelsCmdStamped(vel_left=0, vel_right=0)
        dtros._wheels_publisher.publish(message)

        if self._target_stall in [Stall.ONE, Stall.TWO]:
            TurnRightTask(R=0, angular_velocity=3, tolerance=0.6).execute(dtros)
        else:
            TurnLeftTask(R=0, angular_velocity=3, tolerance=0.6).execute(dtros)
        
    def updateError(self):
        lines = []
        lines.extend(self._yellow_lines) 
        lines.extend(self._white_lines)
        if lines:
            angles = []
            horiz_scores = []
            for rho, theta in lines:
                angle_deg = np.degrees(theta)
                angles.append(angle_deg)

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
    
class ForwardParkingTask(FinalBehaviorMainTask):
    def __init__(self, target_stall):
        self._target_stall = target_stall
        self.detector = dt_apriltags.Detector(families="tag36h11")
        self._bridge = CvBridge()
        self._target_tag_error = 0
        self._white_line_error = 0

        self._error_last = 0

        self._tag_perimeter = 0

    def onStart(self, dtros):
        vehicle_name = os.environ["VEHICLE_NAME"]
        self._raw_image_topic = f"/{vehicle_name}/camera_node/image/compressed"
        dtros._sub_raw_image = rospy.Subscriber(self._raw_image_topic, CompressedImage, self.callback_raw_image)

    def callback_raw_image(self, msg):
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        undistort = ImageOperations.undistort(image)
        undistort_gray = ImageOperations.getGrayscale(undistort)

        # Convert JPEG bytes to CV image
        image = undistort_gray

        height, width = image.shape[:2]

        # If the image is grayscale (single channel), convert it to BGR
        if len(image.shape) == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        # Detect AprilTags (assuming detector works on grayscale, you might need to convert back or detect on the original)
        results = self.detector.detect(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        for r in results:
            if r.tag_id == self._target_stall.value:
            # Extract the bounding box coordinates and convert to integers
                (ptA, ptB, ptC, ptD) = r.corners
                ptA = (int(ptA[0]), int(ptA[1]))
                ptB = (int(ptB[0]), int(ptB[1]))
                ptC = (int(ptC[0]), int(ptC[1]))
                ptD = (int(ptD[0]), int(ptD[1]))

                self._tag_perimeter = (
                    np.linalg.norm(np.array(ptA) - np.array(ptB)) +
                    np.linalg.norm(np.array(ptB) - np.array(ptC)) +
                    np.linalg.norm(np.array(ptC) - np.array(ptD)) +
                    np.linalg.norm(np.array(ptD) - np.array(ptA))
                )

                # Draw the bounding box of the AprilTag detection in green
                cv2.line(image, ptA, ptB, (0, 255, 0), 2)
                cv2.line(image, ptB, ptC, (0, 255, 0), 2)
                cv2.line(image, ptC, ptD, (0, 255, 0), 2)
                cv2.line(image, ptD, ptA, (0, 255, 0), 2)

                # Draw the center of the AprilTag
                (cX, cY) = (int(r.center[0]), int(r.center[1]))
                cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)

                # Draw the tag id in green on the image
                cv2.putText(image, str(r.tag_id), (ptA[0], ptA[1] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                self._target_tag_error = (r.center[0] - (width // 2))*0.005
                print(self._target_tag_error)
                cv2.imshow("Detection", image)
                cv2.waitKey(1)
                return
            
        self._target_tag_error = 0
        
        cv2.imshow("Detection", image)
        cv2.waitKey(1)

    def runTask(self, dtros):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._tag_perimeter >= 300:
                message = WheelsCmdStamped(vel_left=0, vel_right=0)
                dtros._wheels_publisher.publish(message)
                break
            
            _, error_last_updated, message = PIDOperations.getForwardPIDWheelMsg(base_velocity=0.3, error_last=self._error_last, integration_stored=0, error=self._target_tag_error, proportional_gain=1, derivative_gain=1, integral_gain=0, integral_saturation=0)
            
            self._error_last = error_last_updated

            dtros._wheels_publisher.publish(message)
            
            rate.sleep()

class LaneFollowing(FinalBehaviorMainTask):
    def __init__(self, base_velocity=0.3, debug=True):
        self._bridge = CvBridge()
        self._error_last = 0
        self._error = 0
        self._integration_stored = 0
        self._base_velocity = base_velocity
        self._homography = None
        self._undistorted = None
        self._mask_white = None
        self._mask_yellow = None
        self._debug = debug

    def onStart(self, dtros):
        vehicle_name = os.environ["VEHICLE_NAME"]
        self._raw_image_topic = f"/{vehicle_name}/camera_node/image/compressed"
        dtros._sub_raw_image = rospy.Subscriber(self._raw_image_topic, CompressedImage, self.callback_raw_image)

    def callback_raw_image(self, msg):
        try:
            # Convert compressed image
            cv_image = self._bridge.compressed_imgmsg_to_cv2(msg)
            
            # Set image dimensions if not set
            if self._image_height is None or self._image_width is None:
                self._image_height, self._image_width = cv_image.shape[:2]
            
            # Convert to HSV for better color detection
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Detect red lines (intersection)
            red_lower = np.array([0, 50, 50])
            red_upper = np.array([10, 255, 255])
            red_mask = cv2.inRange(hsv, red_lower, red_upper)
            
            # Detect blue (leader Duckiebot)
            blue_lower = np.array([100, 50, 50])
            blue_upper = np.array([130, 255, 255])
            blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
            
            # Check if at intersection
            red_pixels = np.sum(red_mask > 0)
            self._at_intersection = red_pixels > self._detection_threshold
            
            # Find leader position
            blue_pixels = np.sum(blue_mask > 0)
            if blue_pixels > self._detection_threshold:
                # Get centroid of blue area
                M = cv2.moments(blue_mask)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    self._last_known_position = (cx, cy)
                    self._leader_lost = False
                    self._following_active = True
            else:
                self._leader_lost = True
            
            # Update LED state
            if self._following_active:
                if self._leader_lost:
                    self.set_led_pattern(dtros, 'searching')
                else:
                    self.set_led_pattern(dtros, 'following')
            else:
                self.set_led_pattern(dtros, 'inactive')
            
            # Store direction at intersection
            if self._at_intersection and not self._leader_lost:
                # Determine turn direction based on leader position
                image_center = cv_image.shape[1] // 2
                if self._last_known_position:
                    if self._last_known_position[0] < image_center - 100:
                        self._last_known_direction = 'left'
                    elif self._last_known_position[0] > image_center + 100:
                        self._last_known_direction = 'right'
                    else:
                        self._last_known_direction = 'straight'
            
        except Exception as e:
            rospy.logerr(f"Error in image processing: {str(e)}")

    def isTimeToStop(self):
        return False
    
    def runTask(self, dtros):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.isTimeToStop():
                break
            integration_stored_updated, error_last_updated, message = PIDOperations.getForwardPIDWheelMsg(base_velocity=self._base_velocity, error_last=self._error_last, integration_stored=self._integration_stored, error=self._error, proportional_gain=0.0000002, derivative_gain=0.0000002, integral_gain=0.0000002, integral_saturation=500000)
            self._error_last = error_last_updated
            self._integration_stored = integration_stored_updated
            dtros._wheels_publisher.publish(message)
            rate.sleep()

class FindBrokenBot(LaneFollowing):
    def __init__(self, detection_threshold):
        super().__init__()
        self._duckiebot_detected = False
        self._detection_threshold = detection_threshold

    def callback_raw_image(self, msg):
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        undistorted = ImageOperations.undistort(image)
        homography = ImageOperations.getHomography(undistorted)
        mask_white = ImageOperations.getWhiteMask(homography)
        mask_yellow = ImageOperations.getYellowMask(homography)
        mask_blue = ImageOperations.getDuckiebotBlueMask(undistorted)

        if MaskOperations.getActiveCount(mask_blue) > self._detection_threshold:
            self._duckiebot_detected = True
        
        yellow_error = MaskOperations.computeErrorInAxisX(mask=mask_yellow, target_x=100, pixel_value=1)
        white_error = MaskOperations.computeErrorInAxisX(mask=mask_white, target_x=489, pixel_value=1)

        self._error = yellow_error + white_error

    def isTimeToStop(self):
        return self._duckiebot_detected


class SwitchLanesUntilSafe(LaneFollowing):
    def __init__(self, detection_threshold):
        super().__init__()
        self._duckie_detected_time_stamp = None
        self._detection_threshold = detection_threshold

    def callback_raw_image(self, msg):
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        undistorted = ImageOperations.undistort(image)
        homography = ImageOperations.getHomography(undistorted)
        mask_white = ImageOperations.getWhiteMask(homography)
        mask_yellow = ImageOperations.getYellowMask(homography)

        mask_blue = ImageOperations.getDuckiebotBlueMask(undistorted)

        print(MaskOperations.getActiveCount(mask_blue))

        if MaskOperations.getActiveCount(mask_blue) > self._detection_threshold:
            self._duckie_detected_time_stamp = time.time()

        mask_yellow[:, :90] = 0
        yellow_error = MaskOperations.computeErrorInAxisX(mask=mask_yellow, target_x=489, pixel_value=1)
        white_error = MaskOperations.computeErrorInAxisX(mask=mask_white, target_x=100, pixel_value=1)

        if yellow_error > 100000 and white_error < -100000:
            self._error = (yellow_error + abs(white_error)) 
        elif yellow_error < -100000 and white_error > 100000:
            self._error = (yellow_error + -1*white_error) 
        else:
            self._error = yellow_error + white_error
    
    def isTimeToStop(self):
        if self._duckie_detected_time_stamp is not None and (time.time() - self._duckie_detected_time_stamp) > 1:
            print("Did not see a duckiebot for 2 seconds ! stopping...")
            return True
        return False
    
class LaneFollowUntilIntersection(LaneFollowing):
    def __init__(self, base_velocity, debug=True):
        super().__init__(base_velocity=base_velocity, debug=debug)
        self._red_mask = None

    def callback_raw_image(self, msg):
        super().callback_raw_image(msg)
        self._red_mask = ImageOperations.getRedMask(self._homography)

        if self._debug == True:
            cv2.imshow("Homography Red Mask", self._red_mask)
            cv2.waitKey(1)
    
    def isTimeToStop(self):
        if self._debug == True:
            print(MaskOperations.getActiveCenter(self._red_mask)[1])
        if self._red_mask is not None and MaskOperations.getActiveCount(self._red_mask) > 10000 and MaskOperations.getActiveCenter(self._red_mask)[1] > 300:
            return True
        else:
            return False

class DriveOverRedline(LaneFollowUntilIntersection):
    def __init__(self, base_velocity=0.5, debug=True):
        super().__init__(base_velocity=base_velocity, debug=debug)

    def runTask(self, dtros):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._red_mask is not None and MaskOperations.getActiveCount(self._red_mask) < 10000:
                break
            message = WheelsCmdStamped(vel_left=self._base_velocity, vel_right=self._base_velocity)
            dtros._wheels_publisher.publish(message)
            rate.sleep()

class Stop(FinalBehaviorMainTask):
    def __init__(self, stop_time=1):
        self._stop_time = stop_time
        self._stop_start_stamp = None

    def onStart(self, dtros):
        pass

    def runTask(self, dtros):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._stop_start_stamp is None:
                message = WheelsCmdStamped(vel_left=0, vel_right=0)
                dtros._wheels_publisher.publish(message)
                self._stop_start_stamp = time.time()
                continue
            if time.time() - self._stop_start_stamp > self._stop_time:
                break
            message = WheelsCmdStamped(vel_left=0, vel_right=0)
            dtros._wheels_publisher.publish(message)
            rate.sleep()

class TailUntilIntersection(LaneFollowUntilIntersection):
    def __init__(self, base_velocity, tailing_task, debug=True):
        super().__init__(base_velocity=base_velocity, debug=debug)
        self._tailing_task = tailing_task

    def runTask(self, dtros):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            if self._tailing_task.isTargetTooClose():
                message = WheelsCmdStamped(vel_left=0, vel_right=0)
                dtros._wheels_publisher.publish(message)
                continue

            if self.isTimeToStop():
                break

            integration_stored_updated, error_last_updated, message = PIDOperations.getForwardPIDWheelMsg(base_velocity=self._base_velocity, error_last=self._error_last, integration_stored=self._integration_stored, error=self._error, proportional_gain=0.0000002, derivative_gain=0.0000002, integral_gain=0.0000002, integral_saturation=500000)
            self._error_last = error_last_updated
            self._integration_stored = integration_stored_updated
            dtros._wheels_publisher.publish(message)
            rate.sleep()

class TailingLeftTurn(TurnLeftTask):
    def __init__(self, tailing_task, debug=True):
        super().__init__()
        self._tailing_task = tailing_task
        self._debug=debug

    def runTask(self, dtros):
        precision = self._precision
        tolerance = self._tolerance
        target_radian = self._radians
        angular_velocity = self._angular_velocity
        R = self._R

        if self._debug == True:
            msg = f""" Running a curve task ... target_angle : {target_radian}, R: {R}, precision: {precision}, tolerance: {tolerance} """
            rospy.loginfo(msg)

        rate = rospy.Rate(precision)

        v_l = (R - self._l*3) * angular_velocity
        v_r = (R + self._l*3) * angular_velocity

        while not rospy.is_shutdown():
            message = WheelsCmdStamped(vel_left=v_l, vel_right=v_r)
            total_change_angle = ( self._distance_right - self._distance_left ) / (2*self._l)

            if self._debug == True:
                msg = f""" total_change_angle: {total_change_angle}, target_radian {target_radian}"""
                rospy.loginfo(msg)
            
            if self._tailing_task.isTargetTooClose():
                message = WheelsCmdStamped(vel_left=0, vel_right=0)
                dtros._wheels_publisher.publish(message)
                continue

            if abs(total_change_angle) >= (abs(target_radian) - tolerance):
                message = WheelsCmdStamped(vel_left=0, vel_right=0)
                dtros._wheels_publisher.publish(message)
                break

            dtros._wheels_publisher.publish(message)
            rate.sleep()

        if self._debug == True:
            msg = f""" total_change_angle: {total_change_angle}, target_radian {target_radian}"""
            rospy.loginfo(msg)


class TailingRightTurn(TurnRightTask):
    def __init__(self, tailing_task, debug=True):
        super().__init__()
        self._tailing_task = tailing_task
        self._debug=debug

    def runTask(self, dtros):
        precision = self._precision
        tolerance = self._tolerance
        target_radian = self._radians
        angular_velocity = self._angular_velocity
        R = self._R

        if self._debug == True:
            msg = f""" Running a curve task ... target_angle : {target_radian}, R: {R}, precision: {precision}, tolerance: {tolerance} """
            rospy.loginfo(msg)

        rate = rospy.Rate(precision)

        v_l = (R + self._l*3) * angular_velocity
        v_r = (R - self._l*3) * angular_velocity

        while not rospy.is_shutdown():
            message = WheelsCmdStamped(vel_left=v_l, vel_right=v_r)
            total_change_angle = ( self._distance_right - self._distance_left ) / (2*self._l)

            if self._debug == True:
                msg = f""" total_change_angle: {total_change_angle}, target_radian {target_radian}"""
                rospy.loginfo(msg)
            
            if self._tailing_task.isTargetTooClose():
                message = WheelsCmdStamped(vel_left=0, vel_right=0)
                dtros._wheels_publisher.publish(message)
                continue

            if abs(total_change_angle) >= (abs(target_radian) - tolerance):
                message = WheelsCmdStamped(vel_left=0, vel_right=0)
                dtros._wheels_publisher.publish(message)
                break

            dtros._wheels_publisher.publish(message)
            rate.sleep()

        if self._debug == True:
            msg = f""" total_change_angle: {total_change_angle}, target_radian {target_radian}"""
            rospy.loginfo(msg)

class Tailing(FinalBehaviorMainTask):
    def __init__(self, detection_threshold=1000):
        self._detection_threshold = detection_threshold
        self._bridge = CvBridge()
        
        # State tracking
        self._last_known_position = None  # (x, y) in image coordinates
        self._last_known_direction = None  # 'left', 'right', or 'straight'
        self._at_intersection = False
        self._leader_lost = False
        self._following_active = False
        
        # Image dimensions (will be set in first callback)
        self._image_width = None
        self._image_height = None
        
        # LED colors
        self._led_colors = {
            'following': ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),  # Green when following
            'searching': ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),  # Yellow when lost
            'inactive': ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)    # Off when not following
        }

    def onStart(self, dtros):
        vehicle_name = os.environ["VEHICLE_NAME"]
        
        # Camera subscription
        camera_topic = f"/{vehicle_name}/camera_node/image/compressed"
        dtros._sub_camera = rospy.Subscriber(camera_topic, CompressedImage, self.callback_raw_image)
        
        # LED control
        led_topic = f"/{vehicle_name}/led_emitter_node/led_pattern"
        dtros._led_publisher = rospy.Publisher(led_topic, LEDPattern, queue_size=1)
        
        # Wheels control
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        dtros._wheels_publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

    def set_led_pattern(self, dtros, state):
        pattern = LEDPattern()
        pattern.header = Header()
        pattern.header.stamp = rospy.Time.now()
        color = self._led_colors[state]
        
        # Set all LEDs to the same color
        pattern.rgb_vals = [color] * 5
        dtros._led_publisher.publish(pattern)

    def callback_raw_image(self, msg):
        try:
            # Convert compressed image
            cv_image = self._bridge.compressed_imgmsg_to_cv2(msg)
            
            # Set image dimensions if not set
            if self._image_height is None or self._image_width is None:
                self._image_height, self._image_width = cv_image.shape[:2]
            
            # Convert to HSV for better color detection
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Detect red lines (intersection)
            red_lower = np.array([0, 50, 50])
            red_upper = np.array([10, 255, 255])
            red_mask = cv2.inRange(hsv, red_lower, red_upper)
            
            # Detect blue (leader Duckiebot)
            blue_lower = np.array([100, 50, 50])
            blue_upper = np.array([130, 255, 255])
            blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
            
            # Check if at intersection
            red_pixels = np.sum(red_mask > 0)
            self._at_intersection = red_pixels > self._detection_threshold
            
            # Find leader position
            blue_pixels = np.sum(blue_mask > 0)
            if blue_pixels > self._detection_threshold:
                # Get centroid of blue area
                M = cv2.moments(blue_mask)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    self._last_known_position = (cx, cy)
                    self._leader_lost = False
                    self._following_active = True
            else:
                self._leader_lost = True
            
            # Update LED state
            if self._following_active:
                if self._leader_lost:
                    self.set_led_pattern(dtros, 'searching')
                else:
                    self.set_led_pattern(dtros, 'following')
            else:
                self.set_led_pattern(dtros, 'inactive')
            
            # Store direction at intersection
            if self._at_intersection and not self._leader_lost:
                # Determine turn direction based on leader position
                image_center = cv_image.shape[1] // 2
                if self._last_known_position:
                    if self._last_known_position[0] < image_center - 100:
                        self._last_known_direction = 'left'
                    elif self._last_known_position[0] > image_center + 100:
                        self._last_known_direction = 'right'
                    else:
                        self._last_known_direction = 'straight'
            
        except Exception as e:
            rospy.logerr(f"Error in image processing: {str(e)}")

    def isTargetTooClose(self):
        # Implement safe following distance logic
        if not self._last_known_position:
            return False
        
        # If the leader is in the bottom third of the image, they're too close
        return self._last_known_position[1] > (self._image_height * 2/3)

    def runTask(self, dtros):
        rate = rospy.Rate(10)
        base_speed = 0.3
        
        while not rospy.is_shutdown():
            if self._leader_lost:
                if self._at_intersection and self._last_known_direction:
                    # Execute remembered turn
                    if self._last_known_direction == 'left':
                        TurnLeftTask().execute(dtros)
                    elif self._last_known_direction == 'right':
                        TurnRightTask().execute(dtros)
                    # Reset direction after executing turn
                    self._last_known_direction = None
                else:
                    # Continue straight but slower when searching
                    msg = WheelsCmdStamped(vel_left=base_speed*0.7, vel_right=base_speed*0.7)
                    dtros._wheels_publisher.publish(msg)
            else:
                if self._at_intersection:
                    # Slow down at intersections
                    msg = WheelsCmdStamped(vel_left=base_speed*0.5, vel_right=base_speed*0.5)
                elif self.isTargetTooClose():
                    # Slow down if too close
                    msg = WheelsCmdStamped(vel_left=base_speed*0.3, vel_right=base_speed*0.3)
                else:
                    # Normal following speed
                    msg = WheelsCmdStamped(vel_left=base_speed, vel_right=base_speed)
                    
                # Adjust steering based on leader position
                if self._last_known_position:
                    image_center = self._image_width // 2
                    error = (self._last_known_position[0] - image_center) / image_center
                    steering_factor = 0.3  # Adjust this for steering sensitivity
                    
                    msg.vel_left -= error * steering_factor
                    msg.vel_right += error * steering_factor
                
                dtros._wheels_publisher.publish(msg)
            
            rate.sleep()
