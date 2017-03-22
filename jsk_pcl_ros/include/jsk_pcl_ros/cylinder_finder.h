// -*- mode: c++ -*-
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
 * cylinder_finder.h
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef JSK_PCL_ROS_CYLINDER_FINDER_H__
#define JSK_PCL_ROS_CYLINDER_FINDER_H__

#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <string>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <jsk_topic_tools/connection_based_nodelet.h>
#include <jsk_pcl_ros/CylinderFinderConfig.h>
#include <jsk_recognition_msgs/Cylinder.h>
#include <jsk_recognition_msgs/CylinderArray.h>
//#include <jsk_recognition_utils/time_util.h>
#include <sensor_msgs/PointCloud2.h>


namespace jsk_pcl_ros
{
  class CylinderFinder: public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef CylinderFinderConfig Config;
  protected:
    virtual void onInit();
    virtual void configCallback(Config &config, uint32_t level);
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void segment(const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void segmentFromPoints(const geometry_msgs::PolygonStamped::ConstPtr& msg);

    boost::mutex mutex_;
    Eigen::Vector3f hint_axis_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    ros::Subscriber sub_cloud_;
    ros::Subscriber sub_points_;
    ros::Publisher pub_cylinder_;
    ros::Publisher pub_cylinder_array_;
    ros::Publisher pub_cylinder_with_failure_;
    ros::Publisher pub_cylinder_array_with_failure_;
    ros::Publisher pub_inliers_;
    ros::Publisher pub_coefficients_;
    ros::Publisher pub_pose_stamped_;

    std::string algorithm_;
    double min_radius_;
    double max_radius_;
    double eps_hint_angle_;
    double outlier_threshold_;
    bool use_hint_;
    bool use_normal_;
    int max_iterations_;
    int min_size_;
  };
} // namespace


#endif // JSK_PCL_ROS_CYLINDER_FINDER_H__
