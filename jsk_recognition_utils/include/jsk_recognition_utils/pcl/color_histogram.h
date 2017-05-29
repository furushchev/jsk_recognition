/*
 * color_histogram.h
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef COLOR_HISTOGRAM_H__
#define COLOR_HISTOGRAM_H__

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
    COSINE = 0,
    INTERSECTION,
    KL_DIVERGENCE,
    BATTA
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
                        const int& h_bin_step=10, const int& s_bin_step=10)
  {
    int bin_size = h_bin_step * s_bin_step;
    if (policy == HUE) bin_size = h_bin_step;
    else if (policy == SATURATION) bin_size = s_bin_step;

    histogram.resize(bin_size, 0.0f);

    // histogram
    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
      pcl::PointXYZHSV p = cloud.points[i];
      int h_bin = getBin(p.h, h_bin_step, 0.0, 360.0);
      int s_bin = getBin(p.s, s_bin_step, 0.0, 1.0);

      int index = h_bin_step * s_bin + h_bin;
      if (policy == HUE) index = h_bin;
      else if (policy == SATURATION) index = s_bin;
      histogram[index] += 1.0f;
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
                       double degree,
                       const int& h_bin_step=10, const int& s_bin_step=10)
  {
    int base_offset = std::floor(degree / 360.0 * h_bin_step);
    output.resize(input.size(), 0.0);

    for (size_t h = 0; h < h_bin_step; ++h)
      for (size_t s = 0; s < s_bin_step; ++s)
      {
        int offset = h + base_offset > h_bin_step ? h + base_offset - h_bin_step : h + base_offset;
        int index = h_bin_step * s + h;
        int out_index = h_bin_step * s + h + offset;
        output[out_index] = input[index];
      }
  }
}

#endif // COLOR_HISTOGRAM_H__
