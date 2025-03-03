#!/usr/bin/env python3
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
import os
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA

class LEDColor(Enum):
    Red = [1.0, 0.0, 0.0]
    Green = [0.0, 1.0, 0.0]
    Blue = [0.0, 0.0, 1.0]
    Yellow = [1.0, 1.0, 0.0]
    Teal = [0.0, 1.0, 1.0]
    Magenta = [1.0, 0.0, 1.0]
    Off = [0.0, 0.0, 0.0]

class LEDPosition(Enum):
    ALL = set(range(0,5))
    BackLeft = set([4])
    BackRight = set([3])
    FrontLeft = set([0])
    FrontRight = set([2])

class LEDControl(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(DPathService, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        # construct subscriber
        self._vehicle_name = os.environ["VEHICLE_NAME"]
        state_topic = f"/{self._vehicle_name}/state"
        self.sub = rospy.Subscriber(state_topic, String, self.callback)
        self._led_publisher = rospy.Publisher(f'/{self._vehicle_name}/led_emitter_node/led_pattern', LEDPattern, queue_size=10)

    def callback(self, data):
        if data.data == "Moving":
            green = [0.0,1.0,0.0]
            led_msg = self.create_led_msg(green, LEDPosition.FrontLeft, 0b00001)
            self._led_publisher.publish(led_msg)
        # elif data.data == "Stopping":
        #     red = [1.0,0.0,0.0]
        #     led_msg = self.create_led_msg(red)
        #     self._led_publisher.publish(led_msg)
        # elif data.data == "Exiting":
        #     off = [0.0,0.0,0.0]
        #     led_msg = self.create_led_msg(off)
        #     self._led_publisher.publish(led_msg)
        # else:
        #     raise ValueError("Received unknown data")
        rospy.loginfo("I heard '%s'", data.data)
    
    def create_led_msg(self, colors, indexes, freq_mask):
        led_msg = LEDPattern()

        led_msg.frequency_mask = freq_mask

        rgba = ColorRGBA()
        rgba.r = colors[0]
        rgba.g = colors[1]
        rgba.b = colors[2]
        rgba.a = 1.0

        off_color = ColorRGBA()
        off_color.r = 0.0
        off_color.g = 0.0
        off_color.b = 0.0
        off_color.a = 1.0

        for i in range(5):
            led_msg.rgb_vals.append(
                rgba if i in indexes else off_color
            )
        
        return led_msg

if __name__ == '__main__':
    # create the node
    node = LEDControl(node_name='led_control_node')
    # keep spinning
    rospy.spin()