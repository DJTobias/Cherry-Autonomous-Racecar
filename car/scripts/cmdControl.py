#!/usr/bin/env python
"""
Copyright (c) 2017 Daniel Tobias

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import rospy
import tf
from geometry_msgs.msg import Twist, TwistStamped
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy
##import joinstates
from Xbox360 import XBox360
import sys, os
import time
from threading import Lock


"""
This node is a python implementation of the jetsoncar_joy.cpp with some added features and functionality  
This node takes in joystick, lidar twist, and neural twist and toggles the control between these inputs for
a final cmd_vel twist that get sent to the Teensy for steering and throttle

Note:
1. Y button trim left
2. B button trim right
3. X button is cruise control, it will read the current throttle value from the right trigger and repeat it until X is pressed again.
4. right trigger must be pressed to once at start to initialize all other commands

WIP
Add jointstate for URDF

"""

class twistControl(object):

    def __init__(self):
        self.throttleInitialized = False
        self.joy_timeout = 2.0
        self.lid_timeout = 0.30
        self.cnn_timeout = 0.1
        self.joy_time = time.time()
        self.lid_time = time.time()
        self.cnn_time = time.time()
        self.controller = XBox360()
        self.joy_cmd = TwistStamped()
        self.lid_cmd = TwistStamped()
        self.cnn_cmd = TwistStamped()
        self.cruiseControl = False
        self.cruiseThrottle = 0.5
        self.steeringAngle = 0.5
        self.throttle = 0.5
        self.trim = 0.0
        ##self.throttleLock = Lock()
        print "cmd_control"
       
        rospy.Subscriber("/imu", Imu, self.imuCB, queue_size=5)
        rospy.Subscriber("/lidar_twist", TwistStamped, self.lidarTwistCB, queue_size=5)
        rospy.Subscriber("/neural_twist", TwistStamped, self.neuralTwistCB, queue_size=5)
        rospy.Subscriber("/joy", Joy, self.joyCB, queue_size=5)
        self.vel_pub = rospy.Publisher("/cmd_vel", TwistStamped, queue_size = 1) 
        #self.state_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)
        self.sound = rospy.Publisher("/sound_server/speech_synth", String, queue_size=1)        
        rospy.init_node ('cmd_control',anonymous=True)
        rate = rospy.Rate(66)
        while not rospy.is_shutdown():
            self.cmdRouter()
            rate.sleep()
            

    def imuCB(self, imu):
        try: 
            quaternion = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]
        except Exception as e:
            print(e)
            ##TODO update jointstate

    def lidarTwistCB(self, lidarTwist):
        try: 
            self.lid_cmd.twist = lidarTwist.twist
            if self.cruiseControl == True:
                
                if lidarTwist.twist.linear.x == 0.5:
                    self.lid_cmd.twist.linear.x = 0.5
                else:
                    self.lid_cmd.twist.linear.x = self.cruiseThrottle
            else:
                self.lid_cmd.twist.linear.x = self.throttle
            self.lid_time = time.time()
        except Exception as b:
            print(b)


    def neuralTwistCB(self, neuralTwist):
        try:
            self.cnn_cmd.twist = neuralTwist.twist
            
            if self.cruiseControl == True:
                self.cnn_cmd.twist.linear.x = self.cruiseThrottle
            else:
                self.cnn_cmd.twist.linear.x = self.throttle
            self.cnn_time = time.time()
        except Exception as a:
            print(a)
        

    def joyCB(self, joy):
        try:
            self.joystick = joy
            self.controller.update(joy)
            events = self.controller.buttonEvents()
            if 'X_pressed' in events:
                self.cruiseControl = not self.cruiseControl ##toggle cruise control
                self.cruiseThrottle = self.throttle   
            if self.joystick.axes[5] != 0.0 and not self.throttleInitialized:
                self.throttleInitialized = True  
            if self.throttleInitialized:
                self.steeringAngle = self.trim + (1-(((self.joystick.axes[0])/2.0)+0.5))  ## normed steering
                self.joy_cmd.twist.angular.z = self.steeringAngle
                self.throttle = (((1.0-((0.5*self.joystick.axes[5])+0.5))/2.0)+ 0.5)      ## normed throttle
                self.joy_cmd.twist.linear.x = self.throttle         
                if 'Y_pressed' in events:
                    self.trim = self.trim - 0.0025                ##trim left
                if 'B_pressed' in events:
                    self.trim = self.trim + 0.0025                ##trim right
                if 'X_pressed' in events:
                    self.cruiseControl = not self.cruiseControl   ##toggle cruise control
                    self.cruiseThrottle = self.throttle           ##set throttle speed
                if joy.buttons[0] == 1:                           ##Reverse, A button hit twice and hold down
                    self.joy_cmd.twist.linear.z = 1
                else: 
                    self.joy_cmd.twist.linear.z = 0
                if self.cruiseControl == True: 
                    self.joy_cmd.twist.linear.x = self.cruiseThrottle
                else: 
                    self.joy_cmd.twist.linear.x = self.throttle 
                rospy.loginfo(self.cruiseControl)
                self.joy_time = time.time()
                #rospy.loginfo(self.joy_time)
                #self.vel_pub.publish(self.joy_cmd)
            else:
                pass
            if 'X_pressed' in events:
                    self.cruiseControl = not self.cruiseControl ##toggle cruise control
                    self.cruiseThrottle = self.throttle 
                
        except Exception as f:
            print(f)
            

    def cmdRouter(self):     
        if self.joy_time+self.joy_timeout >= time.time():  ## If last joy message was recieved less than 2 seconds ago Joystick controlled      
            rospy.loginfo("Joy cmd")
            rospy.loginfo(self.joy_time+self.joy_timeout)
            rospy.loginfo(time.time())
            self.vel_pub.publish(self.joy_cmd) 
        elif self.lid_time+self.lid_timeout >= time.time(): ##else if last lidar twist was recieved less than 0.3 seconds ago lidar evasion controlled
            self.vel_pub.publish(self.lid_cmd)  
            rospy.loginfo("LIDAR cmd")
        elif self.cnn_time+self.cnn_timeout >= time.time(): ## else if last neural twist was recieved less than 0.1 seconds ago neural net controlled
            self.vel_pub.publish(self.cnn_cmd)
            rospy.loginfo("CNN cmd")
        else:                                               ##else if no command, neutral throttle and steering
            self.joy_cmd.twist.linear.x = 0.5
            self.joy_cmd.twist.angular.z = 0.5
            self.vel_pub.publish(self.joy_cmd)
            rospy.loginfo("Joy cmd else")

if __name__ == '__main__':
    try:
        twistControl()
    except rospy.ROSInterruptException:
        pass
