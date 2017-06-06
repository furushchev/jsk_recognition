#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import cv2
from cv_bridge import CvBridge
import numpy as np
from matplotlib import pyplot as plt
import rospy
from jsk_recognition_msgs.msg import ColorHistogram
from sensor_msgs.msg import Image
from jsk_pcl_ros_utils.cfg import ColorHistogramConfig as Config


class VisualizeColorHistogram(object):
    def __init__(self):
        self.histogram_policy = rospy.get_param("~histogram_policy", 2)
        self.canvas_width = rospy.get_param("~canvas_width", 640)
        self.canvas_height = rospy.get_param("~canvas_height", 480)

        self.cv_bridge = CvBridge()
        self.hsv_color_map = plt.cm.get_cmap('hsv')
        self.figure = plt.figure(figsize=(self.canvas_width, self.canvas_height))
        self.plot = self.figure.add_subplot(111)
        self.hsv_color_map_2d = self.get_hsv_color_map_2d()

        self.pub_image = rospy.Publisher("output", Image, queue_size=1)
        self.sub_image = rospy.Subscriber("histogram", ColorHistogram, self.callback)

    def get_hsv_color_map_2d(self):
        hsv_map = np.zeros((self.canvas_height, self.canvas_width, 3), np.uint8)
        h, s = np.indices(hsv_map.shape[:2])
        hsv_map[:, :, 0] = h / float(self.canvas_height) * 180.0
        hsv_map[:, :, 1] = s / float(self.canvas_width)  * 256.0
        hsv_map[:, :, 2] = 255
        hsv_map = cv2.cvtColor(hsv_map, cv2.COLOR_HSV2BGR)
        return hsv_map

    def callback(self, msg):
        if self.histogram_policy == Config.ColorHistgoram_HUE:
            imgmsg = self.plot_hist_hue(msg.histogram)
        elif self.histogram_policy == Config.ColorHistogram_SATURATION:
            imgmsg = self.plot_hist_saturation(msg.histogram)
        elif self.histogram_policy == Config.ColorHistogram_HUE_AND_SATURATION:
            imgmsg = self.plot_hist_hs(msg.histogram)
        else:
            rospy.logerr("Invalid histogram policy")
            return
        self.pub_image.publish(imgmsg)

    def image_from_plot(self, plot):
        self.figure.canvas.draw()
        w, h = self.figure.canvas.get_width_height()
        buf = np.fromstring(self.figure.canvas.tostring_rgb(), dtype=np.uint8)
        buf.shape = (h, w, 3)
        buf = np.roll(buf, 1, axis=-1)
        try:
            return self.cv_bridge.cv2_to_imgmsg(buf, "bgr8")
        except:
            return None

    def plot_hist_hue(self, hist):
        self.plot.clear()
        bin_size = len(hist)
        bin_step = 360.0 / bin_size
        x = np.arange(360.0, step=bin_step)
        bars = self.plot.bar(x, hist, width=bin_step)
        cs = np.arange(0.0, 1.0, 1.0 / bin_size)
        for c, b in zip(cs, bars):
            b.set_facecolor(self.hsv_color_map(c))
        self.plot.xlim(0, 360.0)
        return self.image_from_plot(self.plot)

    def plot_hist_saturation(self, hist):
        self.plot.clear()
        bin_size = len(hist)
        bin_step = 256.0 / bin_size
        x = np.arange(256.0, step=bin_step)
        self.plot.bar(x, hist, width=bin_step)
        self.plot.xlim(0, 256.0)
        return self.image_from_plot(self.plot)

    def plot_hist_hs(self, hist):
        self.plot.clear()
        bin_size = len(hist)
        h = np.array(hist).reshape(bin_size / 2, bin_size / 2)
        h = np.clip(h, 0, 1)
        img = self.hsv_color_map_2d * h[:,:,np.newaxis] / 255.0
        return img

if __name__ == '__main__':
    rospy.init_node("vars(isualize_color_histogram")
    v = VisualizeColorHistogram()
    rospy.spin()
