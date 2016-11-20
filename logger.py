#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
#sys.path.insert(1, '/usr/local/lib/python2.7/site-packages')
import cv2
print cv2.__version__
import time
from threading import Lock


class logger(object):

    def __init__(self):
        print "logger"
        self.twist = None
        self.twistLock = Lock()
        self.bridge = CvBridge()
        rospy.Subscriber("/stream", Image, self.streamCB)
        rospy.Subscriber("/jetsoncar_teleop_joystick/cmd_vel", Twist, self.cmd_velCB)
        rospy.init_node('logger',anonymous=True)
        rospy.spin()

    def streamCB(self, pic):
        rospy.loginfo("Pic Recieved")
        try:
            cv2image = self.bridge.imgmsg_to_cv2(pic)
            #timestamp = str(pic.header.stamp.secs)
            timestamp = str(time.time())
            if self.twist is not None:
                fname = None
                with self.twistLock:
                    fname = timestamp + '_' + str(self.twist.linear.x) + '_' + str(self.twist.angular.z)
                cv2.imwrite("/home/ubuntu/TrainingIMG/"+fname+".jpg",cv2image)
        except CvBridgeError as e:
            print(e)

    def cmd_velCB(self, msg):
        rospy.loginfo("Linear: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        rospy.loginfo("Angular: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
        with self.twistLock:
            self.twist = msg


if __name__ == '__main__':
    try:
        logger()
    except rospy.ROSInterruptException:
        pass
