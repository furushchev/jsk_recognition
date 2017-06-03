#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import cv_bridge
import cv2
import rospy
from jsk_recognition_msgs.msg import ColorHistogram
from sensor_msgs.msg import Image


class VisualizeColorHistogram(object):
    def __init__(self):
        self.cv_bridge = cv_bridge.CvBridge()
        self.bin_size = rospy.get_param("~bin_size", 10)
        self.histogram_policy = rospy.get_param("~histogram_policy", 2)
        self.sub_image = rospy.Subscriber("histogram", ColorHistogram, self.callback)
        self.pub_image = rospy.Publisher("output", Image, queue_size=1)

    def callback(self, msg):
        pass

    def plot_hist_1d(self, hist):
        pass

if __name__ == '__main__':
    rospy.init_node("vars(isualize_color_histogram")
    v = VisualizeColorHistogram()
    rospy.spin()
