"""
Created by: 
Daniel Tobias
Ryan Dellana
Webcam located as video4
Uses Gstreamer to grab frames from the camera then publishes them as stream
Gstreamer is potentially accelerated on TX1 need to find source of this but it does run faster than default USB
"""

#!/usr/bin/env python
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

def gst_to_opencv(gst_buffer):
    return np.ndarray((480,640,3), buffer=gst_buffer.extract_dup(0, gst_buffer.get_size()), dtype=np.uint8)

def gCamera():
    print "gCamera"
    bridge = CvBridge()
    pub = rospy.Publisher('stream', Image, queue_size=10)
    rospy.init_node('gcam',anonymous=True)
    Gst.init(None)

    pipe = Gst.parse_launch("""v4l2src device=/dev/video4 ! video/x-raw, width=640, height=480,format=(string)BGR ! appsink sync=false max-buffers=2 drop=true name=sink emit-signals=true""")
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
