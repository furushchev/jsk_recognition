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
                        const bin_size=10)
  {
    if (policy == HUE_AND_SATURATION) {
      std::vector<float> saturation;
      computeColorHistogram(cloud, histogram, HUE, bin_size);
      computeColorHistogram(cloud, saturation, SATURATION, bin_size);
      histogram.insert(histogram.end(), saturation.begin(), saturation.end());
      return;
    }

    histogram.resize(bin_size, 0.0);

    // histogram
    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
      pcl::PointXYZHSV p = cloud.points[i];
      int h_bin = getBin(p.h, bin_size, 0.0, 360.0);
      int s_bin = getBin(p.s, bin_size, 0.0, 1.0);

      int index = 0;
      if (policy == HUE) histogram[h_bin] += 1.0f;
      else if (policy == SATURATION) histogram[s_bin] += 1.0f;
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

  inline double
  compareHistogram(const jsk_recognition_msgs::ColorHistogram &input,
                   const jsk_recognition_msgs::ColorHistogram &reference,
                   const double rotate_degree=0.0,
                   const ComparePolicy policy=KL_DIVERGENCE,
                   const int& h_bin_step=10, const int& s_bin_step=10)
  {
    if (input.histogram.size() != h_bin_step * s_bin_step ||
        reference.histogram.size() != h_bin_step * s_bin_step) {
      ROS_ERROR("Mismatch histogram length");
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
                   const ComparePolicy policy=KL_DIVERGENCE,
                   const int& h_bin_step=10, const int& s_bin_step=10)
  {
    double degree = 20.0;
    return std::min(compareHistogram(input, reference, degree, policy, h_bin_step, s_bin_step),
                    std::min(compareHistogram(input, reference,   0.0, policy, h_bin_step, s_bin_step),
                             compareHistogram(input, reference, -degree, policy, h_bin_step, s_bin_step)));
  }

}

#endif // COLOR_HISTOGRAM_H__
