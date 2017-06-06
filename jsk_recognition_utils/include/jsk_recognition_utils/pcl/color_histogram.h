// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/*
 * color_histogram.h
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef JSK_RECOGNITION_UTILS_COLOR_HISTOGRAM_H__
#define JSK_RECOGNITION_UTILS_COLOR_HISTOGRAM_H__

#include <algorithm>
#include <iterator>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace jsk_recognition_utils {
  enum HistogramPolicy {
    HUE = 0,
    SATURATION,
    HUE_AND_SATURATION
  };

  enum ComparePolicy {
    INNER_PRODUCT = 0,
    BHATTACHARYYA,
    INTERSECTION,
    KL_DIVERGENCE
  };

  inline int
  getBin(const double& val, const int& step,
         const double& min, const double& max)
  {
    int idx = std::floor((val - min) / (max - min) * step);
    if (idx >= step) return step - 1;
    else if (idx <= 0) return 0;
    else return idx;
  }

  inline void
  computeColorHistogram(const pcl::PointCloud<pcl::PointXYZHSV>& cloud,
                        std::vector<float>& histogram,
                        const HistogramPolicy policy=HUE_AND_SATURATION,
                        const int bin_size=10)
  {
    if (policy == HUE_AND_SATURATION) histogram.resize(bin_size * bin_size, 0.0f);
    else histogram.resize(bin_size, 0.0f);

    // histogram
    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
      pcl::PointXYZHSV p = cloud.points[i];
      int h_bin = getBin(p.h, bin_size, 0.0, 360.0);
      int s_bin = getBin(p.s, bin_size, 0.0, 1.0);

      if (policy == HUE) histogram[h_bin] += 1.0f;
      else if (policy == SATURATION) histogram[s_bin] += 1.0f;
      else histogram[s_bin * bin_size + h_bin] += 1.0f;
    }

    // normalization
    double sum = 0.0;

    for (size_t i = 0; i < histogram.size(); ++i)
      sum += histogram[i];

    if (sum != 0.0) {
      for (size_t i = 0; i < histogram.size(); ++i) {
        histogram[i] /= sum;
      }
    }
  }

  inline void
  rotateHistogramByHue(const std::vector<float>&input,
                       std::vector<float>& output,
                       const double degree=20.0,
                       const int bin_size=10)
  {
    int offset = std::floor(degree / 360.0 * bin_size);
    output.resize(input.size(), 0.0);

    for (size_t h = 0; h < bin_size; ++h) {
      int hout = h + offset >= bin_size ? h + offset - bin_size : h + offset;
      output[hout] = input[h];
    }

    if (input.size() > bin_size) {
      // copy saturation
      for (size_t s = bin_size; s < bin_size * 2; ++s) {
        output[s] = input[s];
      }
    }
  }

  inline double
  compareHistogram(const jsk_recognition_msgs::ColorHistogram &input,
                   const jsk_recognition_msgs::ColorHistogram &reference,
                   const double rotate_degree=0.0,
                   const int bin_size=10,
                   const ComparePolicy policy=KL_DIVERGENCE)
  {
    if (input.histogram.size() != reference.histogram.size()) {
      ROS_ERROR("Mismatch histogram bin size");
      return -1.0;
    }

    double distance = 0.0;
    switch (policy) {
    case INNER_PRODUCT:
      for (size_t i = 0; i < input.histogram.size(); ++i) {
        distance += input.histogram[i] * reference.histogram[i];
      }
      break;
    case BHATTACHARYYA:
      for (size_t i = 0; i < input.histogram.size(); ++i) {
        distance += std::sqrt(input.histogram[i] * reference.histogram[i]);
      }
      break;
    case INTERSECTION:
      for (size_t i = 0; i < input.histogram.size(); ++i) {
        distance += std::min(input.histogram[i], reference.histogram[i]);
      }
      break;
    case KL_DIVERGENCE:
    default:
      for (size_t i = 0; i < input.histogram.size(); ++i) {
        distance += input.histogram[i] * std::log((input.histogram[i] + 1e-9) / (reference.histogram[i] + 1e-9));
      }
      distance = std::exp(-1.0 * distance / 3.0);
      break;
    }
    return distance;
  }

  inline double
  compareHistogram(const jsk_recognition_msgs::ColorHistogram &input,
                   const jsk_recognition_msgs::ColorHistogram &reference,
                   const int bin_size=10,
                   const ComparePolicy policy=KL_DIVERGENCE)
  {
    double degree = 20.0;
    double dcw  = compareHistogram(input, reference, degree, bin_size, policy);
    double d0   = compareHistogram(input, reference, 0.0, bin_size, policy);
    double dccw = compareHistogram(input, reference, -degree, bin_size, policy);
    return std::min(dcw, std::min(d0, dccw));
  }
}

#endif // JSK_RECOGNITION_UTILS_COLOR_HISTOGRAM_H__
