#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import math
import cv2
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
import rospy
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_msgs.msg import ColorHistogram, ColorHistogramArray
from sensor_msgs.msg import Image
from jsk_pcl_ros_utils.cfg import VisualizeColorHistogramConfig as Config


# matplotlib outputs image with size 800x600 by default
IMG_WIDTH=800
IMG_HEIGHT=600


class VisualizeColorHistogram(ConnectionBasedTransport):
    def __init__(self):
        super(VisualizeColorHistogram, self).__init__()

        self.dyn_server = Server(Config, self.config_callback)

        self.cv_bridge = CvBridge()
        self.hsv_color_map = plt.cm.get_cmap('hsv')
        self.hsv_color_map_2d = self.get_hsv_color_map_2d()

        self.pub_image = self.advertise("~output", Image, queue_size=1)

    def config_callback(self, config, level):
        self.histogram_policy = config.histogram_policy
        self.histogram_index = config.histogram_index
        self.histogram_scale = config.histogram_scale
        return config

    def subscribe(self):
        self.sub_histogram = rospy.Subscriber("~input", ColorHistogram,
                                              self.callback, queue_size=1)
        self.sub_histogram_array = rospy.Subscriber("~input/array", ColorHistogramArray,
                                                    self.callback_array, queue_size=1)

    def unsubscribe(self):
        self.sub_histogram.unregister()
        self.sub_histogram_array.unregister()

    def get_hsv_color_map_2d(self):
        hsv_map = np.zeros((IMG_HEIGHT, IMG_WIDTH, 3), np.uint8)
        h, s = np.indices(hsv_map.shape[:2])
        hsv_map[:, :, 0] = h / float(IMG_HEIGHT) * 180.0
        hsv_map[:, :, 1] = 125.0
        hsv_map[:, :, 2] = s / float(IMG_WIDTH)  * 256.0
        hsv_map = cv2.cvtColor(hsv_map, cv2.COLOR_HLS2BGR)
        return hsv_map

    def callback(self, msg):
        if self.histogram_policy == Config.VisualizeColorHistogram_HUE:
            img = self.plot_hist_hue(msg.histogram)
        elif self.histogram_policy == Config.VisualizeColorHistogram_HUE_AND_SATURATION:
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

    def image_from_plot(self):
        fig = plt.gcf()
        fig.canvas.draw()
        w, h = fig.canvas.get_width_height()
        img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8)
        fig.clf()
        img.shape = (h, w, 3)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img

    def plot_hist_hue(self, hist):
        bin_size = len(hist) - 2
        bin_step = 360.0 / bin_size
        x = np.arange(360.0, step=bin_step)
        bars = plt.bar(x, hist, width=bin_step)
        cs = np.arange(0.0, 1.0, 1.0 / bin_size)
        for c, b in zip(cs, bars[:-2]):
            b.set_facecolor(self.hsv_color_map(c))
        b.set_facecolor(
        plt.xlim(0, 360.0)
        return self.image_from_plot()

    def plot_hist_saturation(self, hist):
        bin_size = len(hist)
        bin_step = 256.0 / bin_size
        x = np.arange(256.0, step=bin_step)
        plt.bar(x, hist, width=bin_step)
        plt.xlim(0, 256.0)
        return self.image_from_plot()

    def plot_hist_hs(self, hist):
        bin_size = int(math.sqrt(len(hist))) - 2
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
