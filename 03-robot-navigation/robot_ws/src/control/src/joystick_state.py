#!/usr/bin/env python
import numpy as np
import cv2
import roslib
import rospy
import tf
import struct
import math
import time
from std_msgs.msg import Header
from sensor_msgs.msg import Image, LaserScan
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool, SetBoolRequest

class JOYSTICK():
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing " %(self.node_name))
		# self.pub_info = rospy.Publisher('/MSG_NAME', SubTInfo, queue_size=1)
		self.topic_name = rospy.get_param('~joy_name','/joy_teleop/joy')
		self.sub_odom = rospy.Subscriber(self.topic_name, Joy, self.joy_cb, queue_size=1)
		self.anto_srv = rospy.ServiceProxy('/emergency_stop', SetBool)
			
	def joy_cb(self, msg):
		if msg.buttons[1] == 1:
			req = SetBoolRequest()
			req.data = False
			resp = self.anto_srv(req)
		elif msg.buttons[3] == 1:
			req = SetBoolRequest()
			req.data = True
			resp = self.anto_srv(req)

if __name__ == '__main__':
	rospy.init_node('JOYSTICK')
	foo = JOYSTICK()
	rospy.spin()