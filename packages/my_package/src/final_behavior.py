#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped, LEDPattern
import math
import time
from std_msgs.msg import ColorRGBA, Header


class FinalBehaviorMain(DTROS):
    def __init__(self, node_name, tasks):
        super(FinalBehaviorMain, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self.initSensorReadings()

        self._l = 0.050

        self._resolution = 135

        self._tasks = tasks

        self._vehicle_name = os.environ["VEHICLE_NAME"]
        self._radius = rospy.get_param(f'/{self._vehicle_name}/kinematics_node/radius', 0.0318)

        wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        state_topic = f"/{self._vehicle_name}/state"
        self._wheels_publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        self._led_publisher = rospy.Publisher(f'/{self._vehicle_name}/led_emitter_node/led_pattern', LEDPattern, queue_size=10)
        self._state_publisher = rospy.Publisher(state_topic, String, queue_size=1)
        
        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
        self._sub_left_wheel = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left_wheel)
        self._sub_right_wheel = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right_wheel)
    
    def initSensorReadings(self):
        self._ticks_left = None
        self._ticks_right = None

        self._distance_left = 0
        self._distance_right = 0

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

    def run(self):
        for task in self._tasks:

            if not isinstance(task,FinalBehaviorMainTask):
                raise ValueError("task not recognized")
            
            self.initSensorReadings()

            print("Running " + str(task.__class__.__name__))

            task.execute(self)
            
        rospy.signal_shutdown(reason="tasks complete")

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._wheels_publisher.publish(stop)

class FinalBehaviorMainTask():
    def execute(self, dtros):
        self.run(dtros)
    
    def run(self, dtros):
        raise Exception("run() must be overriden")

class TailingTask(FinalBehaviorMainTask):
    pass

class AprilTagBehaviorTask(FinalBehaviorMainTask):
    pass

class ObstacleBehaviorTask(FinalBehaviorMainTask):
    pass

class ParkingTask(FinalBehaviorMainTask):
    pass



# Turn right and left need to be different classes
class TurnRightTask(FinalBehaviorMainTask):
    def __init__(self, precision=40, tolerance=0.04, radians=math.pi/2, angular_velocity=1, R=0.4445):
        super().__init__()
        self._precision = precision
        self._tolerance = tolerance
        self._radians = radians
        self._angular_velocity=angular_velocity
        self._R = R

    def run(self, dtros):
        
        precision = self._precision
        tolerance = self._tolerance
        target_radian = self._radians
        angular_velocity = self._angular_velocity
        R = self._R

        msg = f""" Running a curve task ... target_angle : {target_radian}, R: {R}, precision: {precision}, tolerance: {tolerance} """
        rospy.loginfo(msg)

        rate = rospy.Rate(precision)

        v_r = (R - dtros._l*3) * angular_velocity
        v_l = (R + dtros._l*3) * angular_velocity

        message = WheelsCmdStamped(vel_left=v_l, vel_right=v_r)

        while not rospy.is_shutdown():
            total_change_angle = ( dtros._distance_right - dtros._distance_left ) / (2*dtros._l)
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

    def run(self, dtros):
        
        precision = self._precision
        tolerance = self._tolerance
        target_radian = self._radians
        angular_velocity = self._angular_velocity
        R = self._R

        msg = f""" Running a curve task ... target_angle : {target_radian}, R: {R}, precision: {precision}, tolerance: {tolerance} """
        rospy.loginfo(msg)

        rate = rospy.Rate(precision)

        v_l = (R - dtros._l*3) * angular_velocity
        v_r = (R + dtros._l*3) * angular_velocity

        message = WheelsCmdStamped(vel_left=v_l, vel_right=v_r)

        while not rospy.is_shutdown():
            total_change_angle = ( dtros._distance_right - dtros._distance_left ) / (2*dtros._l)
            msg = f""" total_change_angle: {total_change_angle}, target_radian {target_radian}"""
            rospy.loginfo(msg)
            if abs(total_change_angle) >= (abs(target_radian) - tolerance):
                break

            dtros._wheels_publisher.publish(message)
            rate.sleep()

        msg = f""" total_change_angle: {total_change_angle}, target_radian {target_radian}"""
        rospy.loginfo(msg)

        
if __name__ == "__main__":
    tasks = [
        TurnLeftTask(R=0.7)
    ]
    node = FinalBehaviorMain(node_name="final_behavior_main_node", tasks=tasks)
    node.run()
    rospy.spin()