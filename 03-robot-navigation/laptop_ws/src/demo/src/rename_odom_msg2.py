#!/usr/bin/env python
import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from duckietown_msgs.msg import Pose2DStamped, GlobalPoseArray

def msg_cb(msg):
    for pose in msg.poses:
        odom_msg = Odometry()
        odom_msg.header.frame_id = "map"
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.pose.pose.position.x = pose.pose.x
        odom_msg.pose.pose.position.y = pose.pose.y 
        q = tf.transformations.quaternion_from_euler(0, 0, pose.pose.theta + np.pi/2.)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        if pose.bot_id == 427 and bot_name=="mmbot14": 
            loc_mmbot_pub.publish(odom_msg)
        elif pose.bot_id == 426 and bot_name=="mmbot16": 
            loc_mmbot_pub.publish(odom_msg)
        # if pose.bot_id == 427: loc_mmbot14_pub.publish(odom_msg)
        # elif pose.bot_id == 426: loc_mmbot16_pub.publish(odom_msg)
        # elif pose.bot_id == 432: loc_obstacle1_pub.publish(odom_msg)
        # elif pose.bot_id == 433: loc_obstacle2_pub.publish(odom_msg)

if __name__ == '__main__':
    rospy.init_node('Odom_Msg_Rename_Node')
    
    global loc_mmbot_pub
    global bot_name
    bot_name = rospy.get_param("~bot")
    if bot_name == "mmbot14":
        rospy.Subscriber('/nctuece/pose_optimization/bot_global_poses_optimized', GlobalPoseArray, msg_cb)
    elif bot_name == "mmbot16":
        rospy.Subscriber('/erickietop/pose_optimization/bot_global_poses_optimized', GlobalPoseArray, msg_cb)

    loc_mmbot_pub = rospy.Publisher('wt_odom', Odometry, queue_size=2)
    # loc_mmbot14_pub = rospy.Publisher('/mmbot14/wt_odom', Odometry, queue_size=2)
    # loc_mmbot16_pub = rospy.Publisher('/mmbot16/wt_odom', Odometry, queue_size=2)
    # loc_obstacle1_pub = rospy.Publisher('/obj1_pose', Odometry, queue_size=2)
    # loc_obstacle2_pub = rospy.Publisher('/obj2_pose', Odometry, queue_size=2)
    rospy.spin()
