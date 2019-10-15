#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

class Estop(object):
    def __init__(self):
        self.es_lora = Bool()
        self.es_xbee = Bool()
        self.enable_xbee = True
        self.sub_lora = rospy.Subscriber('e_stop_lora',Bool,self.cb_lora,queue_size=1)
        self.sub_xbee = rospy.Subscriber('e_stop_xbee',Bool,self.cb_xbee,queue_size=1)
        self.pub_estop = rospy.Publisher('e_stop',Bool,queue_size=1)
        self.sub_joy = rospy.Subscriber('joy_teleop/joy',Joy,self.cb_joy,queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1),self.process)

    def cb_joy(self,joy_msg):
        if (joy_msg.buttons[4] == 1):
            self.enable_xbee = True
            rospy.logerr('enable xbee estop')
        elif (joy_msg.buttons[5] == 1):
            self.enable_xbee = False
            rospy.logerr('cut off xbee estop')

    def process(self,event):
        if self.enable_xbee:
            self.pub_estop.publish(self.es_lora or self.es_xbee)
        else:
            self.pub_estop.publish(self.es_lora)

    def cb_lora(self,msg):
        self.es_lora = msg.data

    def cb_xbee(self,msg):
        self.es_xbee = msg.data

    def on_shutdown(self):
        self.pub_estop.publish(False)


if __name__ == "__main__":
    rospy.init_node('estop_control')
    estop = Estop()
    rospy.on_shutdown(estop.on_shutdown)
    rospy.spin()
    