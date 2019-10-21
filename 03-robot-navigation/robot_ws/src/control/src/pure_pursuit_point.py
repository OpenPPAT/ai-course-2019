#!/usr/bin/env python
import numpy as np
import cv2
import roslib
import rospy
import tf
import struct
import math
import time
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, LaserScan
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point, Twist
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry, Path
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from control.cfg import pos_PIDConfig, ang_PIDConfig
from std_srvs.srv import SetBool, SetBoolResponse
robot = None
DIS_THRES = 0.1 # Distance threshold
ANGLE_THRES = np.radians(45.0)
path = []
pub_point = rospy.Publisher('/pursue_point', PoseStamped, queue_size=10)
use_odom = None

def odom_cb(msg):
	global robot, path, pub_point
	pose = PoseStamped()
	# Get robot pose now (x, y)
	robot = [msg.pose.pose.position.x, msg.pose.pose.position.y]
	# Hold only pose that distance with robot greater than threshold
	path_hold = []
	if path == []:
		return
	for p in path:
		start = False
		if start or distanceBtwnPoints(p[0], p[1], robot[0], robot[1]) > DIS_THRES:
			start = True
			path_hold.append(p)
	path[:] = path_hold[:]
	if path == []:
		return
	# Publish first index
	pose.header = msg.header
	pose.pose.position.x = path[0][0]
	pose.pose.position.y = path[0][1]
	pub_point.publish(pose)

def path_cb(msg):
	rospy.loginfo('get planning path!')
	global robot, path, use_odom
	if robot is None and use_odom:
		return
	elif not use_odom:
		robot = [0, 0]
	path = []
	start = False
	if len(msg.poses) == 1 and msg.poses[0].pose.position.x == 0.0 and msg.poses[0].pose.position.y == 0.0:
			pose = PoseStamped()
			pose.pose.position.x = 0.0
			pose.pose.position.y = 0.0
			pub_point.publish(pose)
			return
	for pose in msg.poses:
		p = [pose.pose.position.x, pose.pose.position.y]
		too_close = distanceBtwnPoints(p[0], p[1], robot[0], robot[1]) < DIS_THRES
		if too_close: rospy.loginfo("too close")
		# Not add to path if too close 
		if start or not too_close:
			start = True
			path.append(p)
	if not use_odom:
		pose = PoseStamped()
		pose.header = msg.header
		path_hold = []
		for p in path:
			if distanceBtwnPoints(p[0], p[1], robot[0], robot[1]) > DIS_THRES:
				path_hold.append(p)
		path[:] = path_hold[:]
		if path == []:
			return
		
		if abs(np.arctan2(path[0][1], path[0][0])) >= ANGLE_THRES:
			#print "Angle greater than threshold"
			for pose_ in msg.poses:
				p = [pose_.pose.position.x, pose_.pose.position.y]
				if distanceBtwnPoints(p[0], p[1], robot[0], robot[1]) >= 0.3:
					pose.pose.position.x = p[0]
					pose.pose.position.y = p[1]
					break
		else:
			pose.pose.position.x = path[0][0]
			pose.pose.position.y = path[0][1]
		# print pose
		pub_point.publish(pose)
'''
def cb_arrive(msg):
	# Clear path if arrive
	global path
	if msg.data == True:
		path = []
		print "Arrive signal received, clear path list"
'''
def distanceBtwnPoints(x1, y1, x2, y2):
	return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def main():
	rospy.init_node('pub_point', anonymous=True)
	global use_odom
	use_odom = rospy.get_param('use_odom')
	print "use_odom is: ", use_odom
	rospy.Subscriber("planning_path", Path, path_cb, queue_size=1)
	if use_odom:
		rospy.Subscriber('/pozyx_odom', Odometry, odom_cb, queue_size = 1, buff_size = 2**24)
		robot = [0, 0]
	#rospy.Subscriber('arrive', Bool, cb_arrive, queue_size = 1)
	rospy.spin()

if __name__ == '__main__':
	main()
