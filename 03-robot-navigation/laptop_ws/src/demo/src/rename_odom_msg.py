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

        if pose.bot_id == 439 and bot_name=="super_pi01": 
            pub_bot_location.publish(odom_msg)
        elif pose.bot_id == 436 and bot_name=="super_pi02": 
            pub_bot_location.publish(odom_msg)
        elif pose.bot_id == 404 and bot_name=="super_pi03": 
            pub_bot_location.publish(odom_msg)
        elif pose.bot_id == 405 and bot_name=="super_pi04": 
            pub_bot_location.publish(odom_msg)
        elif pose.bot_id == 434 and bot_name=="super_pi05": 
            pub_bot_location.publish(odom_msg)
        elif pose.bot_id == 408 and bot_name=="super_pi06": 
            pub_bot_location.publish(odom_msg)
        elif pose.bot_id == 400 and bot_name=="super_pi07": 
            pub_bot_location.publish(odom_msg)
        elif pose.bot_id == 406 and bot_name=="super_pi08": 
            pub_bot_location.publish(odom_msg)
        elif pose.bot_id == 407 and bot_name=="super_pi09": 
            pub_bot_location.publish(odom_msg)

if __name__ == '__main__':
    rospy.init_node('Odom_Msg_Rename_Node')
    
    global pub_bot_location
    global bot_name
    bot_name = rospy.get_param("~bot")
    rospy.Subscriber('/nctuece/pose_optimization/bot_global_poses_optimized', GlobalPoseArray, msg_cb)

    pub_bot_location = rospy.Publisher('wt_odom', Odometry, queue_size=2)
    # loc_mmbot14_pub = rospy.Publisher('/mmbot14/wt_odom', Odometry, queue_size=2)
    # loc_mmbot16_pub = rospy.Publisher('/mmbot16/wt_odom', Odometry, queue_size=2)
    # loc_obstacle1_pub = rospy.Publisher('/obj1_pose', Odometry, queue_size=2)
    # loc_obstacle2_pub = rospy.Publisher('/obj2_pose', Odometry, queue_size=2)
    rospy.spin()
