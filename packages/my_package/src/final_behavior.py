#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from custom_utils.constants import Stall
from custom_utils.final_tasks import *

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

if __name__ == "__main__":
    stall = Stall.TWO
    tasks = [
        # StallAlignmentTask(target_stall=stall, proportional_gain=0.05, derivative_gain=0.05, integral_gain=0, velocity=0.3, integral_saturation=100),
        # ForwardParkingTask(target_stall=stall),
        # FindBrokenBot(detection_threshold=1000),
        # SwitchLanesUntilSafe(detection_threshold=250) # 250 is original for detection_threshold
        Tailing()
    ]
    node = FinalBehaviorMain(node_name="final_behavior_main_node", tasks=tasks)
    node.run()
    rospy.spin()