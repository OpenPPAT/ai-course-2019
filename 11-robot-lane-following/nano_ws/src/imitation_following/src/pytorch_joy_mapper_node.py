#!/usr/bin/env python
from __builtin__ import True
import numpy as np
import rospy
import math
import torch
from torch import nn
import torch.backends.cudnn as cudnn
from torch.optim.lr_scheduler import CosineAnnealingLR, MultiStepLR
import torchvision
import cv2
import os
from torchvision import transforms, utils, datasets
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from os.path import expanduser
home = expanduser("~")


class AlexNet(nn.Module):

    def __init__(self):
        super(AlexNet, self).__init__()
        self.features = nn.Sequential(
            nn.Conv2d(3, 64, kernel_size=11, stride=4, padding=2),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=3, stride=2),
            nn.Conv2d(64, 192, kernel_size=5, padding=2),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=3, stride=2),
            nn.Conv2d(192, 384, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(384, 256, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(256, 256, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=3, stride=2),
        )
        self.avgpool = nn.AdaptiveAvgPool2d((6, 6))
        self.classifier = nn.Sequential(
            nn.Dropout(),
            nn.Linear(256 * 6 * 6, 4096),
            nn.ReLU(inplace=True),
            nn.Dropout(),
            nn.Linear(4096, 4096),
            nn.ReLU(inplace=True),
            nn.Linear(4096, 15),
        )

    def forward(self, x):
        x = self.features(x)
        x = self.avgpool(x)
        x = torch.flatten(x, 1)
        x = self.classifier(x)
        return x


class JoyMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " % (self.node_name))

        self.joy = None
        self.last_pub_msg = None
        self.last_pub_time = rospy.Time.now()

        # initial Pytorch
        self.initial()
        self.bridge = CvBridge()
        self.joy_control = 1
        self.ncs_control = 0
        self.omega = 0
        self.count = 0
        self.model_path = rospy.get_param("model_path")

        # Setup Parameters
        self.v_gain = self.setupParam("~speed_gain", 0.41)
        self.omega_gain = self.setupParam("~steer_gain", 8.3)
        self.bicycle_kinematics = self.setupParam("~bicycle_kinematics", 0)
        self.steer_angle_gain = self.setupParam("~steer_angle_gain", 1)
        self.simulated_vehicle_length = self.setupParam(
            "~simulated_vehicle_length", 0.18)

        # Publications
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_joy_override = rospy.Publisher(
            "~joystick_override", BoolStamped, queue_size=1)
        self.pub_parallel_autonomy = rospy.Publisher(
            "~parallel_autonomy", BoolStamped, queue_size=1)
        self.pub_anti_instagram = rospy.Publisher(
            "anti_instagram_node/click", BoolStamped, queue_size=1)
        self.pub_e_stop = rospy.Publisher(
            "wheels_driver_node/emergency_stop", BoolStamped, queue_size=1)
        self.pub_avoidance = rospy.Publisher(
            "~start_avoidance", BoolStamped, queue_size=1)

        # Subscriptions
        #self.image_sub = rospy.Subscriber(
        #    "~image/compressed", CompressedImage, self.img_cb, queue_size=1)
        print("1")
        self.image_sub = rospy.Subscriber(
            "/super_pi06/camera_node/image/compressed", CompressedImage, self.img_cb, queue_size=1)
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)

        # timer
        self.param_timer = rospy.Timer(
            rospy.Duration.from_sec(1.0), self.cbParamTimer)
        self.has_complained = False

        self.state_parallel_autonomy = False
        self.state_verbose = False

        pub_msg = BoolStamped()
        pub_msg.data = self.state_parallel_autonomy
        pub_msg.header.stamp = self.last_pub_time
        self.pub_parallel_autonomy.publish(pub_msg)

     

    # Load model
    def initial(self):
        self.model = AlexNet()
        state_dict = torch.load(self.model_path)
        self.model.load_state_dict(state_dict)

       

    def img_cb(self, data):
        self.dim = (101, 101)  # (width, height)
        self.count += 1
        if self.count == 6:
            self.count = 0
            try:
                # convert image_msg to cv format
                np_arr = np.fromstring(data.data, np.uint8)
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                img = cv2.resize(img, self.dim)

                self.data_transform = transforms.Compose([
                    transforms.ToTensor(),
                    transforms.Normalize(mean=[0.5, 0.5, 0.5],
                                         std=[1, 1, 1]),
                ])
                img = self.data_transform(img)
                self.images = torch.unsqueeze(img,0)
                
                
                device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
                self.images = self.images.to(device)
                self.model = self.model.to(device)
                self.output = self.model(self.images)
                self.top1 = self.output.argmax()
                self.omega = (int(self.top1) * (2/14.) - 1) * 8.5
                print(self.omega)

                if self.ncs_control == 1:
                    car_cmd_msg = Twist2DStamped()
                    car_cmd_msg.v = 1 * 0.36
                    car_cmd_msg.omega = self.omega
                    self.pub_car_cmd.publish(car_cmd_msg)

            except CvBridgeError as e:
                print(e)

    def cbParamTimer(self, event):
        self.v_gain = rospy.get_param("~speed_gain", 1.0)
        self.omega_gain = rospy.get_param("~steer_gain", 10)

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparancy
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def cbJoy(self, joy_msg):
        self.joy = joy_msg
        self.publishControl()
        self.processButtons(joy_msg)

    def publishControl(self):
        if self.joy_control == 1:
            car_cmd_msg = Twist2DStamped()
            car_cmd_msg.header.stamp = self.joy.header.stamp
            # Left stick V-axis. Up is positive
            car_cmd_msg.v = self.joy.axes[1] * self.v_gain
            # Holonomic Kinematics for Normal Driving
            car_cmd_msg.omega = self.joy.axes[3] * self.omega_gain
            self.pub_car_cmd.publish(car_cmd_msg)


# Button List index of joy.buttons array:
# a = 0, b=1, x=2. y=3, lb=4, rb=5, back = 6, start =7,
# logitek = 8, left joy = 9, right joy = 10


    def processButtons(self, joy_msg):
        if (joy_msg.buttons[6] == 1):  # The back button
            override_msg = BoolStamped()
            override_msg.header.stamp = self.joy.header.stamp
            override_msg.data = True
            rospy.loginfo('joystick_control = True')
            self.joy_control = 1
            self.ncs_control = 0
            self.pub_joy_override.publish(override_msg)

        elif (joy_msg.buttons[7] == 1):  # the start button
            override_msg = BoolStamped()
            override_msg.header.stamp = self.joy.header.stamp
            override_msg.data = False
            rospy.loginfo('deep_lanefollowing = True')
            self.joy_control = 0
            self.ncs_control = 1
            self.pub_joy_override.publish(override_msg)

        elif (joy_msg.buttons[3] == 1):
            anti_instagram_msg = BoolStamped()
            anti_instagram_msg.header.stamp = self.joy.header.stamp
            anti_instagram_msg.data = True
            rospy.loginfo('anti_instagram message')
            self.pub_anti_instagram.publish(anti_instagram_msg)


if __name__ == "__main__":
    rospy.init_node("joy_mapper", anonymous=False)
    joy_mapper = JoyMapper()
    rospy.spin()
