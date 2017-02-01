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
##import cPickle as pickle
import numpy as np
import rospy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import sys
import gi
from Xbox360 import XBox360
import cv2
print cv2.__version__
gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst
import time
from threading import Lock


"""
This node saves multiple image topics as pictures and encodes the filename with sequence,time,throttle,steering.
I know what rosbag is.

Note: 
Throttle values 0.0-0.5 = reverse and 0.5-1.0 = forward
Steering values 0.0-0.5 = left and 0.5-1.0 = right, 0.5 is center position
This node is recieving the output topic of drop nodes that are dropping 2 out of 3 frames for the depth and rgb image topics.
The node struggles to record both rgb and depth topic at 30FPS with cv2.imwrite on the TX1 which is why messages are being dropped plus 30FPS seems to be unnecessary
Gstreamer could be used to create a similar function like cv2.imwrite that utilizes the onboard hardware encoders for jpg (nvjpg) this could allow for more FPS or image topics to be saved as jpg
Location of folders where images are saved are /home/ubuntu/DepthIMG/ /home/ubuntu/LidarIMG/ /home/ubuntu/TrainingIMG/

      
"""

class dataRecorder(object):

    def __init__(self):
        print "dataRecorder"
        self.record = False
        self.twist = None
        self.twistLock = Lock()
        self.bridge = CvBridge()
        self.globaltime = None
        self.controller = XBox360()
        ##rospy.Subscriber("/camera/depth/points" , PointCloud2, self.ptcloudCB)
        rospy.Subscriber("/camera/depth/image_rect_raw_drop", Image, self.depthCB)
        rospy.Subscriber("/camera/rgb/image_rect_color_drop", Image, self.streamCB)
        #rospy.Subscriber("/camera/rgb/image_rect_mono_drop", Image, self.streamCB)  ##for black and white images see run.launch and look at the drop fps nodes near the bottom
        rospy.Subscriber("/lidargrid", Image, self.lidargridCB)
        rospy.Subscriber("/cmd_vel", TwistStamped, self.cmd_velCB)
        rospy.Subscriber("/joy", Joy ,self.joyCB)
        self.sound = rospy.Publisher("/sound_server/speech_synth", String, queue_size=1)
        rospy.init_node('dataRecorder',anonymous=True)
        rospy.spin()

    """
    Similar to the other functions except storing a pointcloud2 msg and pickleing it. It is relatively expensive to store a pointcloud2 topic in pickle format.
    """
    ## def ptcloudCB(self, cloud):
    ##     if self.record == True:
    ##         timestamp = str(time.time())
    ##         fname = None
    ##         with self.twistLock:
    ##             fname = timestamp + '_' + str(round(self.twist.linear.x,8)) + '_' + str(round(self.twist.angular.z,8))        
    ##             with open('/home/ubuntu/Clouds/'+fname+'.pickle', 'wb') as f:
    ##                 pickle.dump(cloud,f,protocol=2)

    """
    Receives a Lidar Image and encodes the sequence,timestamp,throttle and steering values into the filename and saves it as a tiff which is lossless.
    Important: Depth images are uint16 and when you imread you must pass it a -1. eg. imread(img,-1) 
    """
    def depthCB(self, depth):
        if self.record == True:
            #rospy.loginfo("depth image recieved")
            try:
                grey = self.bridge.imgmsg_to_cv2(depth)
                rospy.loginfo(grey.dtype)

                if self.twist is not None:
                    fnamedepth = None
                    seq = str(depth.header.seq)
                    timestamp = str(depth.header.stamp)
                    with self.twistLock:
                        fnamedepth = seq + '_' + timestamp + '_' + str(round(self.twist.linear.x,8)) + '_' + str(round(self.twist.angular.z,8))
                    cv2.imwrite("/home/ubuntu/DepthIMG/"+fnamedepth+".tiff",grey)
            except CvBridgeError as x:
                print(x)
        else:
            rospy.loginfo("Not Recording Depth")
        
    """
    Receives a LIDAR image and encodes the sequence,timestamp,throttle and steering values into the filename and saves it as a jpg
    Note: The modified neato_laser_publisher.cpp node is converting the LIDAR data from polar to cartesian coordinates then writing them as white pixels on a black image of size 100x100
    """
    def lidargridCB(self, grid):
        if self.record == True:
            #rospy.loginfo("grid recieved")
            try:
                cv2lidarImg = self.bridge.imgmsg_to_cv2(grid)
                if self.twist is not None:
                    fnamelidar = None
                    timestamp = str(grid.header.stamp)
                    seq = str(grid.header.seq)
                    with self.twistLock:                  
                        fnamelidar = seq + '_' + timestamp + '_' + str(round(self.twist.linear.x,8)) + '_' + str(round(self.twist.angular.z,8))
                    cv2.imwrite("/home/ubuntu/LidarIMG/"+fnamelidar+".jpg",cv2lidarImg)
            except CvBridgeError as r:
                print(r)
        else:
            rospy.loginfo("Not Recording Lidar")

    """
    Receives an Image message and encodes the sequence,timestamp,throttle and steering values into the filename and saves it as a jpg
    """
    def streamCB(self, pic):
        if self.record == True:
            #rospy.loginfo("image recieved")
            try:
                cv2image = self.bridge.imgmsg_to_cv2(pic)
                if self.twist is not None:
                    fname = None
                    seq = str(pic.header.seq)
                    timestamp = str(pic.header.stamp)
                    with self.twistLock:
                        fname = seq + '_' + timestamp + '_' + str(round(self.twist.linear.x,8)) + '_' + str(round(self.twist.angular.z,8))
                    cv2.imwrite("/home/ubuntu/TrainingIMG/"+fname+".jpg",cv2image)
            except CvBridgeError as e:
                print(e)
        else:
            rospy.loginfo("Not Recording Webcam")
    
    """
    Receives a twist msg
    """
    def cmd_velCB(self, msg):
        ##rospy.loginfo("Linear: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        ##rospy.loginfo("Angular: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
        with self.twistLock:
            self.twist = msg.twist

    """
    Toggle recording on or off with start button on the Xbox controller
    """
    def joyCB(self, joy):        
        self.controller.update(joy)
        events = self.controller.buttonEvents()
        if 'start_pressed' in events:
            self.record = not self.record
            if self.record == True:
                self.sound.publish("Recording Started")
            else:
                self.sound.publish("Recording Stopped")


if __name__ == '__main__':
    try:
        dataRecorder()
    except rospy.ROSInterruptException:
        pass
