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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String
from Xbox360 import XBox360
import cv2
from subprocess import call
from sensor_msgs.msg import Joy
import tensorflow as tf
from car_models import cnn_cccccfffff

"""
This node restores a saved TF model that was trained on a host computer

Note:
1. Toggle the publishing of the twist msg with right bumper on the xbox 360 controller
2. Steering output from the trained model is mapped from 0.0 to 1.0 where 0.0 is left, 1.0 is right and 0.5 is center
"""
class runCNN(object):

    def __init__(self):
        rospy.loginfo("runner.__init__")
        # ----------------------------
        self.sess = tf.InteractiveSession()
        self.model = cnn_cccccfffff()
        saver = tf.train.Saver()
        saver.restore(self.sess, "/home/ubuntu/catkin_ws/src/car/scripts/model.ckpt")
        rospy.loginfo("runner.__init__: model restored")
        # ----------------------------
        self.bridge = CvBridge()
        self.netEnable = False
        self.controller = XBox360()
        rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.runCB)
        rospy.Subscriber("/joy", Joy ,self.joyCB)
        self.pub_twist = rospy.Publisher("/neural_twist", TwistStamped, queue_size=1)
        self.sound = rospy.Publisher("/sound_server/speech_synth", String, queue_size=1)
        rospy.init_node('neural_cmd',anonymous=True)
        rospy.spin()


    def runCB(self, pic):
        rospy.loginfo("image recieved")
        if self.netEnable == True: 
            cmd = TwistStamped()
            cv2image = self.bridge.imgmsg_to_cv2(pic)
            cv2image = cv2.resize(cv2image, (200,150), interpolation = cv2.INTER_CUBIC)
            cv2image = cv2image[35:,:,:]
            normed_img = cv2image.astype(dtype=np.float32)/255.0
            #normed_img = np.reshape(normed_img, (115, 200, 1))
            steer = self.model.y_out.eval(session=self.sess, feed_dict={self.model.x: [normed_img], 
                                              self.model.keep_prob_fc1:1.0, self.model.keep_prob_fc2:1.0, 
                                              self.model.keep_prob_fc3:1.0, self.model.keep_prob_fc4:1.0})
            rospy.loginfo("steering angle =" + str((steer[0][0])))
            cmd.twist.angular.z = steer[0][0]
            rospy.loginfo("steering angle =" + str(cmd.twist.angular.z))
            self.pub_twist.publish(cmd)
            
        
    def joyCB(self, joy):        
        self.controller.update(joy)
        events = self.controller.buttonEvents()
        if 'RB_pressed' in events:
            self.netEnable = not self.netEnable
            if self.netEnable == True:
                self.sound.publish("Neural Network On")
            else:
                self.sound.publish("Neural Network Off")

if __name__ == '__main__':
    try:
        runCNN()
    except rospy.ROSInterruptException:
        pass
