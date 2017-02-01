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
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import gi
import sys
sys.path.insert(1, '/usr/local/lib/python2.7/site-packages')
import cv2
print cv2.__version__
gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst

"""
Node that publishes images from the C920 Logitech webcam using Gstreamer

Note: Better performance using gstreamer than cv.capture
      
"""

def gst_to_opencv(gst_buffer):
    return np.ndarray((480,640,3), buffer=gst_buffer.extract_dup(0, gst_buffer.get_size()), dtype=np.uint8)

def gCamera():
    print "gstWebCam"
    bridge = CvBridge()
    video ="video4"
    pub = rospy.Publisher('stream', Image, queue_size=10)
    rospy.init_node('GstWebCam',anonymous=True)
    Gst.init(None)
    pipe = Gst.parse_launch("""v4l2src device=/dev/"""+video+""" ! video/x-raw, width=640, height=480,format=(string)BGR ! appsink sync=false max-buffers=2 drop=true name=sink emit-signals=true""")
    sink = pipe.get_by_name('sink')
    pipe.set_state(Gst.State.PLAYING)
    while not rospy.is_shutdown():
        sample = sink.emit('pull-sample')    
        img = gst_to_opencv(sample.get_buffer())
        try:
            pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    try:
        gCamera()
    except rospy.ROSInterruptException:
        pass




