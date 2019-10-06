#!/usr/bin/env python
import rospy
import math
import time
import tf
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point, Twist, Quaternion
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry

class FakeTrajectoryNode(object):
    """docstring for FakeTrajectoryNode"""
    def __init__(self):
        self.pub_location = rospy.Publisher('wt_odom', Odometry, queue_size = 1)
        self.pub_local_goal = rospy.Publisher("pursue_point", PoseStamped, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    def timer_cb(self, event):
        now = rospy.Time.now()
        odom = Odometry(header=Header(frame_id="map", stamp=now))
        odom.child_frame_id = 'odom'
        odom.pose.pose.position = Point(x=0, y=0, z=0)
        odom.pose.pose.orientation = Quaternion(x=0, y=0, z=0, w=0)
        self.pub_location.publish(odom)

        p = PoseStamped(header=Header(frame_id="map", stamp=now))
        p.pose.position = Point(x=0.2, y=0.2, z=0)
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        p.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.pub_local_goal.publish(p)


if __name__ == '__main__':
    rospy.init_node('fake_trajectory_node')
    node = FakeTrajectoryNode()
    rospy.spin()
