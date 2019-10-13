#!/usr/bin/env python

'''
  PID controller for differential-drive robot 
  Two main things to do:
   * Read data from Arduino (in format v_l v_r theta), use this data to do dead-reckoning
     and publish odom topic -> called by timer with 100Hz
   * Subscribe to twist topic, calculate desired linear velocities of two wheels and use
     PID controller to chase the setpoint, produce pwm value for motors -> called by callback
'''

import serial
import rospy
import tf

from math import sin, cos, isnan
from Adafruit_MotorHAT import Adafruit_MotorHAT
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Empty, EmptyResponse

WIDTH    = 0.172
BASELINE = 1.0
GAIN     = 1.0
K        = 27.0
LIMIT    = 1.0
RADIUS   = 0.032
TRIM     = 0.01

# Check if the list is in range (low, up)
# Params:
#     value_list:  given value list
#    low:         lower bound
#     up:          upper bound
# Output:
#   True: if every value in list in range (low, up)
#   False: otherwise
def in_range(value_list, low, up):
    for i in range(0, len(value_list)):
        if value_list[i] > up or value_list[i] < low:
            return False
    return True

class DiffController(object):
    def __init__(self):
        self.motorhat = Adafruit_MotorHAT(0x60, i2c_bus=1)
        self.left_motor = self.motorhat.getMotor(1)
        self.right_motor = self.motorhat.getMotor(2)

        # Subscriber and publisher
        self.pub_odom = rospy.Publisher('wheel_odom', Odometry, queue_size = 10)
        self.sub_cmd  = rospy.Subscriber('cmd_vel', Twist, self.cmd_cb,  queue_size = 1)

        # Service
        self.reset_odom = rospy.Service('reset_wheel_odom', Empty, self.reset_odom)

        # Left and right wheel velocities
        self.v_r = None
        self.v_l  = None
        # Position variables
        self.heading = 0
        self.x = 0
        self.y = 0

        # Desired velocities
        self.v_d = 0
        self.w_d = 0

        self.k = rospy.get_param('~k', K)
        self.gain = rospy.get_param('~gain', GAIN)
        self.baseline = rospy.get_param('~baseline', BASELINE)
        self.trim = rospy.get_param('~trim', TRIM)
        self.radius = rospy.get_param('~radius', RADIUS)
        self.trim = rospy.get_param('~trim', TRIM)
        self.limit = rospy.get_param('~limit', LIMIT)

        rospy.Timer(rospy.Duration(1/10.0), self.timer_cb) # 100Hz
        self.time = rospy.Time.now()
        rospy.loginfo('[%s] Initialized'  %(rospy.get_name()))


    def reset_odom(self, req):
        self.x = 0
        self.y = 0
        self.heading = 0
        print "Reset wheel odom" 
        return EmptyResponse()


    def timer_cb(self, event):

        # assuming same motor constants k for both motors
        k_r = self.k
        k_l = self.k

        # adjusting k by gain and trim
        k_r_inv = (self.gain + self.trim) / k_r
        k_l_inv = (self.gain - self.trim) / k_l

        omega_r = (self.v_d + 0.5 * self.w_d * self.baseline) / self.radius
        omega_l = (self.v_d - 0.5 * self.w_d * self.baseline) / self.radius

        # conversion from motor rotation rate to duty cycle
        # u_r = (gain + trim) (v + 0.5 * omega * b) / (r * k_r)
        u_r = omega_r * k_r_inv
        # u_l = (gain - trim) (v - 0.5 * omega * b) / (r * k_l)
        u_l = omega_l * k_l_inv

        # limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = max(min(u_r, self.limit), -self.limit)
        u_l_limited = max(min(u_l, self.limit), -self.limit)

        print "u_r: " + str(u_r_limited) + ", u_l: " + str(u_l_limited)
        
        self.motor_motion(u_r_limited*250, u_l_limited*250)


    # sub_cmd callback, get desired linear and angular velocity
    def cmd_cb(self, msg):
        self.v_d = msg.linear.x
        self.w_d = msg.angular.z
        # print "v_d: " + str(self.v_d) + ", w_d: " + str(self.w_d)


    # Send command to motors
    # pwm_r: right motor PWM value
    # pwm_l: left motor PWM value
    def motor_motion(self, pwm_r, pwm_l):
        if pwm_r < 0:
            right_state = Adafruit_MotorHAT.BACKWARD
            pwm_r = -pwm_r
        elif pwm_r > 0:
            right_state = Adafruit_MotorHAT.FORWARD
        else:
            right_state = Adafruit_MotorHAT.RELEASE
        if pwm_l < 0:
            left_state  = Adafruit_MotorHAT.BACKWARD
            pwm_l = -pwm_l
        elif pwm_l > 0:
            left_state = Adafruit_MotorHAT.FORWARD
        else:
            left_state = Adafruit_MotorHAT.RELEASE
                #print "pwm_r: " + str(pwm_r) + ", pwm_l: " + str(pwm_l) 
        self.right_motor.setSpeed(int(pwm_r))
        self.left_motor.setSpeed(int(pwm_l))
        self.right_motor.run(right_state)
        self.left_motor.run(left_state)
        #if pwm_r == 0 and pwm_l == 0:
            #rospy.sleep(1.0)
    
    # Shutdown function, call when terminate
    def shutdown(self):
        self.sub_cmd.unregister()
        rospy.sleep(0.1)
        self.right_motor.setSpeed(0)
        self.left_motor.setSpeed(0)
        self.right_motor.run(Adafruit_MotorHAT.RELEASE)
        self.left_motor.run(Adafruit_MotorHAT.RELEASE)
        del self.motorhat    

if __name__ == '__main__':
    rospy.init_node('diff_controller_node')
    controller = DiffController()
    rospy.on_shutdown(controller.shutdown)
    rospy.spin()
