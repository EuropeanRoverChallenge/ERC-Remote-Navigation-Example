#! /usr/bin/env python

import sys
import os
from datetime import datetime

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2

def save_callback(msg):
    try:
        image = rospy.wait_for_message(msg.data, Image, timeout=5.0)
    except rospy.ROSException as e:
        rospy.logerr("Failed to retrieve image: %s" % (e,))
        return

    try:
        cv2_img = bridge.imgmsg_to_cv2(image, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("Failed to convert image: %s" % (e,))
        return

    img_time = datetime.fromtimestamp(image.header.stamp.secs)
    time_str = img_time.strftime("%Y-%m-%d-%H-%M-%S")
    img_filename = 'image_%s.png' % (time_str,)

    cv2.imwrite(img_filename, cv2_img)

    rospy.loginfo("Saved image from %s topic to %s file", msg.data, 
        os.path.join(os.getcwd(), img_filename))


rospy.init_node('image_saver')

bridge = CvBridge()

rospy.Subscriber("~save", String, save_callback)

rospy.spin()
