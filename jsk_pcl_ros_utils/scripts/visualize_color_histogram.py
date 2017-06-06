#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import math
import cv2
from cv_bridge import CvBridge
import numpy as np
from matplotlib import pyplot as plt
import rospy
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_msgs.msg import ColorHistogram, ColorHistogramArray
from sensor_msgs.msg import Image
from jsk_pcl_ros_utils.cfg import ColorHistogramConfig as Config

# matplotlib outputs image with size 800x600 by default
IMG_WIDTH=800
IMG_HEIGHT=600


class VisualizeColorHistogram(ConnectionBasedTransport):
    def __init__(self):
        super(VisualizeColorHistogram, self).__init__()

        self.histogram_policy = rospy.get_param("~histogram_policy", 2)
        self.histogram_index = rospy.get_param("~histogram_index", 0)
        self.histogram_scale = rospy.get_param("~histogram_scale", 1.0)

        self.cv_bridge = CvBridge()
        self.hsv_color_map = plt.cm.get_cmap('hsv')
        self.figure = plt.figure()
        self.plot = self.figure.add_subplot(111)
        self.hsv_color_map_2d = self.get_hsv_color_map_2d()

        self.pub_image = self.advertise("~output", Image, queue_size=1)

    def subscribe(self):
        self.sub_histogram = rospy.Subscriber("~input", ColorHistogram, self.callback)
        self.sub_histogram_array = rospy.Subscriber("~input/array", ColorHistogramArray, self.callback_array)

    def unsubscribe(self):
        self.sub_histogram.unregister()
        self.sub_histogram_array.unregister()

    def get_hsv_color_map_2d(self):
        hsv_map = np.zeros((IMG_HEIGHT, IMG_WIDTH, 3), np.uint8)
        h, s = np.indices(hsv_map.shape[:2])
        hsv_map[:, :, 0] = h / float(IMG_HEIGHT) * 180.0
        hsv_map[:, :, 1] = s / float(IMG_WIDTH)  * 256.0
        hsv_map[:, :, 2] = 255.0
        hsv_map = cv2.cvtColor(hsv_map, cv2.COLOR_HSV2BGR)
        return hsv_map

    def callback(self, msg):
        if self.histogram_policy == Config.ColorHistogram_HUE:
            img = self.plot_hist_hue(msg.histogram)
        elif self.histogram_policy == Config.ColorHistogram_SATURATION:
            img = self.plot_hist_saturation(msg.histogram)
        elif self.histogram_policy == Config.ColorHistogram_HUE_AND_SATURATION:
            img = self.plot_hist_hs(msg.histogram)
        else:
            rospy.logerr("Invalid histogram policy")
            return
        try:
            pub_msg = self.cv_bridge.cv2_to_imgmsg(img, "bgr8")
            self.pub_image.publish(pub_msg)
        except Exception as e:
            rospy.logerr("Failed to convert image: %s" % str(e))

    def callback_array(self, msg):
        if 0 <= self.histogram_index < len(msg.histograms):
            self.callback(msg.histograms[self.histogram_index])
        else:
            rospy.logerr("histogram index Out-of-index")

    def image_from_plot(self, plot):
        self.figure.canvas.draw()
        w, h = self.figure.canvas.get_width_height()
        buf = np.fromstring(self.figure.canvas.tostring_rgb(), dtype=np.uint8)
        buf.shape = (h, w, 3)
        buf = np.roll(buf, 1, axis=-1)
        return buf

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
        bin_size = int(math.sqrt(len(hist)))
        hist = np.array(hist).reshape(bin_size, bin_size).T
        hist = np.clip(hist * 150 * self.histogram_scale, 0, 1)
        hist = hist[:, :, np.newaxis]
        hist = cv2.resize(hist, (IMG_WIDTH, IMG_HEIGHT))
        hist = hist[:, :, np.newaxis]
        img = self.hsv_color_map_2d * hist
        img = img.astype(np.uint8)
        return img

if __name__ == '__main__':
    rospy.init_node("visualize_color_histogram")
    v = VisualizeColorHistogram()
    rospy.spin()
