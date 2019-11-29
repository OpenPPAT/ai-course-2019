#!/usr/bin/python

# This file is used for extracting images from one or more bag files.
# Before run this file, remember to create a folder to store the images and 2 txt files which will store the 
# image name with its corresponding omega.
# Then change the folder name to the one you created in line 52 & 53.
# In this file, I open three rosbags in order and transform all of it into training data. 
# If you have more bags or less, just add/delete the bag part.
# Code from 57-94 is the same as 95-132 and 133-170, only the name of .bag is different
# After all set, run this file with 'python bag2txt.py'

#PKG = 'beginner_tutorials'
import roslib   #roslib.load_manifest(PKG)
import rosbag
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import numpy as np

# Reading bag filename from command line or roslaunch parameter.
#import os
#import sys

class ImageCreator():


    def __init__(self):
        self.i = 0
        self.t = 0
        self.n = 0
        self.omega = 0
        self.omega_gain = 8.4
        self.bridge = CvBridge()
        f1 = open('lanefollowing/train.txt', 'w') # change "ele" to the folder name you created/
        f2 = open('lanefollowing/test.txt', 'w') 

        #---bag part---
        with rosbag.Bag('1.bag', 'r') as bag: #open first .bag
            print("1.bag")
            for topic,msg,t in bag.read_messages():
                #print topic
                if topic == "/super_pi02/joy_mapper_node/car_cmd": #change ros_master name from aiplus1 to your duckiebot's name
                    #print topic, msg.header.stamp
                    self.omega = msg.omega
                    self.v = msg.v
                    self.t = 1
                elif topic == "/super_pi02/camera_node/image/compressed":
                    if self.t == 1:
                        try:
                            #print topic, msg.header.stamp
                            #cv_image = self.bridge.imgmsg_to_cv2(msg)
                            np_arr = np.fromstring(msg.data, np.uint8)
                            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                        except CvBridgeError as e:
                            print(e)
                        #timestr = "%.6f" %  msg.header.stamp.to_sec()
                        image_name = str(self.n)+ ".jpg" 
                        cv2.imwrite("lanefollowing/"+image_name, cv_image)
                        
                        if self.i == 9:
                            f2.write("joystick/"+image_name+" "+str(self.omega)+" "+str(self.v)+"\n")
                            self.i = 0
                        else:                             
                            f1.write("joystick/"+image_name+" "+str(self.omega)+" "+str(self.v)+"\n")
                            self.i += 1
                        print("image crop:",self.n)
                        self.n += 1
                        self.t = 0

        #---bag part---

        f1.close()
        f2.close()  

if __name__ == '__main__':

    #rospy.init_node(PKG)

    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException:
        pass
