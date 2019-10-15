#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

class visualizer(object):
    def __init__(self):
        self.pub_marker = rospy.Publisher("~pose_visualizer", Marker, queue_size=1)
        self.sub_pose = rospy.Subscriber("/pozyx_node/local_tag_pose", PoseStamped, self.pose_cb, queue_size=1)
    def pose_cb(self, pose_msg):
        #print "pose cb"
        marker_msg = Marker()
        marker_msg.header.frame_id = "uwb_pose"
        marker_msg.header.stamp = rospy.Time()
        marker_msg.ns = "my_namespace"
        marker_msg.id = 0
        marker_msg.action = Marker.ADD
        marker_msg.type = Marker.ARROW
        marker_msg.pose.position.x = pose_msg.pose.position.x/1000
        marker_msg.pose.position.y = pose_msg.pose.position.y/1000
        marker_msg.pose.position.z = pose_msg.pose.position.z/1000

        marker_msg.pose.orientation.x = pose_msg.pose.orientation.x
        marker_msg.pose.orientation.y = pose_msg.pose.orientation.y
        marker_msg.pose.orientation.z = pose_msg.pose.orientation.z
        marker_msg.pose.orientation.w = pose_msg.pose.orientation.w
        marker_msg.scale.x = 0.6
        marker_msg.scale.y = 0.1
        marker_msg.scale.z = 0.1
        

        marker_msg.color.r = 1.0
        marker_msg.color.g = 1.0
        marker_msg.color.b = 1.0
        marker_msg.color.a = 1.0

        marker_msg.lifetime = rospy.Duration()
        self.pub_marker.publish(marker_msg)



if __name__ == '__main__':
	rospy.init_node('pose_visualization',anonymous=False)
	node = visualizer()
	rospy.spin()
