#!/usr/bin/env python
"""
Copyright (c) 2017 Daniel Tobias, Ryan Dellana

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
import numpy as np
import rospy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy
from cv_bridge import CvBridge, CvBridgeError
from Xbox360 import XBox360
import sys, os
import cv2
print cv2.__version__
import time
from threading import Lock

"""
This node subscribes to the lidargrid image topic which is basicially a birds eye view of the Neato XV-11 LIDAR's /scan topic.  
It then uses the image and checks for white pixels(points) in predefined regions of the image. 
If a region contains an white pixel it will publish a twist message with a steering angle or throttle value to try and avoid the object or stop.
This node's intention was to be used as a primitive layer to help prevent the car from crashing into a wall under the Neural Net's control.

Future plans:
This node is very simple currently but could be expanded upon with fusion of IMU data. The regions that are being defined to look for collision could be modeled based on current speed and steering angle as a sort of basic look ahead for obstacles function
Note: 
The region function is defined using the TF coordinates which is a right handed coordinate system see http://www.pirobot.org/blog/0018/ 
Toggle on or off with select button on Xbox 360 controller   
      
"""

lidarRadius = 5.0  ##meters

class evasion(object):

    def __init__(self):
        self.evadeSet = False
        self.controller = XBox360()
        self.bridge = CvBridge()
        self.throttle = 0
        self.grid_img = None
        ##self.throttleLock = Lock()
        print "evasion"
        rospy.Subscriber("/lidargrid", Image, self.gridCB, queue_size=1)
        rospy.Subscriber("/cmd_vel", TwistStamped , self.twistCB , queue_size = 1)
        rospy.Subscriber("/joy", Joy, self.joyCB, queue_size=5)
        self.pub_img = rospy.Publisher("/steering_img", Image)
        self.pub_twist = rospy.Publisher("/lidar_twist", TwistStamped, queue_size=1)
        self.sound = rospy.Publisher("/sound_server/speech_synth", String, queue_size=1)        
        rospy.init_node ('lidar_cmd',anonymous=True)
        rospy.spin()

    def twistCB(self, cmd_vel):
        if self.evadeSet == True:
            try:
                ##rospy.loginfo("cmd_vel Recieved")
                self.throttle = cmd_vel.twist.linear.x
                normed_throttle = (self.throttle*2.0)-1.0
                front_max = 0.3 + 4.5*(normed_throttle**2.5)   ##front region scales with throttle value
                rospy.loginfo('normed_throttle: '+str(normed_throttle) + ' front_max: '+str(front_max))
                front = self.Occupancy(self.grid_img, 0.1, front_max, -0.2,  0.2)    ##(2,0.2)  to (0.5,-0.2)
                right = self.Occupancy(self.grid_img, 0.0, 1, -0.7, -0.2)            ##(2,-0.2) to (0,-0.7)
                left  = self.Occupancy(self.grid_img, 0.0, 1,  0.2,  0.7)            ##(2,0.7)  to (0,0.2)
                everywhere = self.Occupancy(self.grid_img, -4.0, 4.0, -4.0, 4.0)
                cmd = TwistStamped()
                #rospy.loginfo(self.throttle)
                cmd.twist.angular.z = 0.5
                cmd.twist.linear.x = -1.0
                if front:
                    cmd.twist.linear.x = 0.5   ##stop
                    self.pub_twist.publish(cmd)                   
                    self.sound.publish("Forward collision detected")       
                elif left:
                    cmd.twist.angular.z = 0.7  ##turn right
                    self.pub_twist.publish(cmd)           
                    self.sound.publish("Left collision detected")                       
                elif right:
                    cmd.twist.angular.z = 0.3  ##turn left
                    self.pub_twist.publish(cmd)
                    self.sound.publish("Right collision detected")
                else:
                    #self.pub_twist.publish(cmd)
                    pass 
            except Exception as f:
                print(f)
        else:
            pass
            ##rospy.loginfo("Not using Evasion")

    def gridCB(self, grid):
        rospy.loginfo("Grid Recieved")
        try:
            self.grid_img = self.bridge.imgmsg_to_cv2(grid)
        except CvBridgeError as e:
            print(e)

    """
    Toggle LIDAR evasion with select button on Xbox 360 Controller
    """
    def joyCB(self, joy):
        self.controller.update(joy)
        events = self.controller.buttonEvents()
        if 'back_pressed' in events:
            self.evadeSet = not self.evadeSet
            rospy.loginfo(self.evadeSet)

    """
    Converts TF to pixels(x1,y1),(x2,y2)
    """
    def Region(self, grid, xmin, xmax, ymin, ymax):
        pixelwidth = grid.shape[0]
        tfwidth = lidarRadius*2.0
        endx = int( pixelwidth * ((xmin*-1.0+5.0)/tfwidth) )
        endy = int( pixelwidth * ((ymin*-1.0+5.0)/tfwidth) )
        startx = int( pixelwidth * ((xmax*-1.0+5.0)/tfwidth) )
        starty = int( pixelwidth * ((ymax*-1.0+5.0)/tfwidth) )
        startx, starty = max(0, startx), max(0, starty)
        endx, endy = min(endx, pixelwidth), min(endy, pixelwidth)
        return (startx, starty, endx, endy)
    """
    checks for occupancy in regions specified then returns a true or false from a sum
    """
    def Occupancy(self, grid, xmin, xmax, ymin, ymax):
        (startx, starty, endx, endy) = self.Region(grid, xmin, xmax, ymin, ymax)
        #rospy.loginfo(str(startx)+" "+str(starty)+" "+str(endx)+" "+str(endy))
        region = grid[startx:endx,starty:endy]
        #rospy.loginfo(str(region.shape))
        sum_ = region.sum()
        #rospy.loginfo(str(sum_))
        if sum_ > 0:
            return True
        else:
            return False
    
if __name__ == '__main__':
    try:
        evasion()
    except rospy.ROSInterruptException:
        pass
