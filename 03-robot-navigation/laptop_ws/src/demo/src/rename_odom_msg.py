#!/usr/bin/env python
import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from duckietown_msgs.msg import Pose2DStamped

def msg_cb(msg):
    odom_msg = Odometry()
    odom_msg.header.frame_id = "map"
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.pose.pose.position.x = msg.x
    odom_msg.pose.pose.position.y = msg.y
    q = tf.transformations.quaternion_from_euler(0, 0, msg.theta + np.pi/2.)
    odom_msg.pose.pose.orientation.x = q[0]
    odom_msg.pose.pose.orientation.y = q[1]
    odom_msg.pose.pose.orientation.z = q[2]
    odom_msg.pose.pose.orientation.w = q[3]
    wt_pub.publish(odom_msg)
    

if __name__ == '__main__':
    rospy.init_node('Odom_Msg_Rename_Node')
    rospy.Subscriber('/nctuece/absolute_from_relative_position/bot_global_poses_for_car', Pose2DStamped, msg_cb)
    global wt_pub
    wt_pub = rospy.Publisher('wt_odom', Odometry, queue_size=2)
    rospy.spin()
