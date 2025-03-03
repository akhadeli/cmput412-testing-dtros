#!/usr/bin/env python3
from enum import Enum
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
import os
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
    

class LEDControl(DTROS):
    def __init__(self, node_name):
        super(LEDControl, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        self._vehicle_name = os.environ["VEHICLE_NAME"]
        state_topic = f"/{self._vehicle_name}/state"
        self.sub = rospy.Subscriber(state_topic, String, self.callback)
        self._led_publisher = rospy.Publisher(f'/{self._vehicle_name}/led_emitter_node/led_pattern', LEDPattern, queue_size=10)
        self.states = [
            State(message_name="Moving", colorPattern=ColorPattern(frontLeft=Colors.Green, frontRight=Colors.Green, backLeft=Colors.Green, backRight=Colors.Green)),
            State(message_name="Stopping", colorPattern=ColorPattern(frontLeft=Colors.Red, frontRight=Colors.Red, backLeft=Colors.Red, backRight=Colors.Red)),
            State(message_name="Turning Right", colorPattern=ColorPattern(frontLeft=Colors.Off, frontRight=Colors.Yellow, backLeft=Colors.Off, backRight=Colors.Yellow)),
            State(message_name="Turning Left", colorPattern=ColorPattern(frontLeft=Colors.Yellow, frontRight=Colors.Off, backLeft=Colors.Yellow, backRight=Colors.Off)),
        ]

    def callback(self, data):
        for state in self.states:
            if state.message_name == data.data:
                self._led_publisher.publish(state.getLedMessage())
                break
        rospy.loginfo("I heard '%s'", data.data)


if __name__ == '__main__':
    # create the node
    node = LEDControl(node_name='led_control_node')
    # keep spinning
    rospy.spin()