// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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
 * cylinder_finder_nodelet.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#define BOOST_PARAMETER_MAX_ARITY 7

#include <jsk_pcl_ros/cylinder_finder.h>
#include <jsk_recognition_utils/pcl_conversion_util.h>
#include <jsk_topic_tools/rosparam_utils.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>


namespace jsk_pcl_ros
{
  void CylinderFinder::onInit()
  {
    DiagnosticNodelet::onInit();
    pcl::console::setVerbosityLevel(pcl::console::L_WARN);

    pnh_->param("use_hint", use_hint_, false);
    if (use_hint_) {
      if (pnh_->hasParam("initial_axis_hint")) {
        std::vector<double> axis;
        jsk_topic_tools::readVectorParameter(*pnh_, "initial_axis_hint", axis);
        if (axis.size() == 3) {
          hint_axis_[0] = axis[0];
          hint_axis_[1] = axis[1];
          hint_axis_[2] = axis[2];
        }
        else {
          hint_axis_[0] = 0;
          hint_axis_[1] = 0;
          hint_axis_[2] = 1;
        }
      }
    }

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&CylinderFinder::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_cylinder_ = advertise<jsk_recognition_msgs::Cylinder>(*pnh_, "output", 1);
    pub_cylinder_array_ = advertise<jsk_recognition_msgs::CylinderArray>(*pnh_, "output/array", 1);
    pub_cylinder_with_failure_ = advertise<jsk_recognition_msgs::Cylinder>(*pnh_, "output/with_failure", 1);
    pub_cylinder_array_with_failure_ = advertise<jsk_recognition_msgs::CylinderArray>(*pnh_, "output/with_failure/array", 1);
    pub_inliers_ = advertise<PCLIndicesMsg>(*pnh_, "output/inliers", 1);
    pub_coefficients_ = advertise<PCLModelCoefficientMsg>(*pnh_, "output/coefficients", 1);
    pub_pose_stamped_ = advertise<geometry_msgs::PoseStamped>(*pnh_, "output/pose", 1);

    onInitPostProcess();
  }

  void CylinderFinder::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);

    min_radius_ = config.min_radius;
    max_radius_ = config.max_radius;
    outlier_threshold_ = config.outlier_threshold;
    max_iterations_ = config.max_iterations;
    min_size_ = config.min_size;
    eps_hint_angle_ = config.eps_hint_angle;
    algorithm_ = config.algorithm;
  }

  void CylinderFinder::subscribe()
  {
    sub_cloud_ = pnh_->subscribe("input", 1,
                                 &CylinderFinder::segment, this);
    // sub_points_ = pnh_->subscribe("input/polygon", 1,
    //                               &CylinderFinder::segmentFromPoints, this);
  }

  void CylinderFinder::unsubscribe()
  {
    sub_cloud_.shutdown();
    // sub_points_.shutdown();
  }
  /*
  void CylinderFinder::segmentFromPoints(const geometry_msgs::PolygonStamped::ConstPtr& msg)
  {
    // convenience callback for polygon
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud
      (new pcl::PointCloud<pcl::PointNormal>);
    for (size_t i = 0; i < msg->polygon.points.size(); i++) {
      geometry_msgs::Point32 p = msg->polygon.points[i];
      pcl::PointNormal pcl_point;
      pcl_point.x = p.x;
      pcl_point.y = p.y;
      pcl_point.z = p.z;
      cloud->points.push_back(pcl_point);
    }
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud);
    ros_cloud.header = msg->header;
    segment(boost::make_shared<sensor_msgs::PointCloud2>(ros_cloud));
  }
  */
  void CylinderFinder::segment(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud_xyz);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg(*msg, *cloud_normal);
 
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::PointNormal> seg;

    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setOptimizeCoefficients (true);
    seg.setInputCloud(cloud_xyz);
    seg.setInputNormals(cloud_normal);
    seg.setRadiusLimits(min_radius_, max_radius_);

    if (algorithm_ == "RANSAC") {
      seg.setMethodType(pcl::SAC_RANSAC);
    }
    else if (algorithm_ == "LMEDS") {
      seg.setMethodType(pcl::SAC_LMEDS);
    }
    else if (algorithm_ == "MSAC") {
      seg.setMethodType(pcl::SAC_MSAC);
    }
    else if (algorithm_ == "RRANSAC") {
      seg.setMethodType(pcl::SAC_RRANSAC);
    }
    else if (algorithm_ == "RMSAC") {
      seg.setMethodType(pcl::SAC_RMSAC);
    }
    else if (algorithm_ == "MLESAC") {
      seg.setMethodType(pcl::SAC_MLESAC);
    }
    else if (algorithm_ == "PROSAC") {
      seg.setMethodType(pcl::SAC_PROSAC);
    }
    else {
      NODELET_ERROR("No algorithm specified. Use RANSAC");
      seg.setMethodType(pcl::SAC_RANSAC);
    }

    seg.setDistanceThreshold (outlier_threshold_);
    seg.setMaxIterations (max_iterations_);

    if (use_hint_) {
      seg.setAxis(hint_axis_);
      seg.setEpsAngle(eps_hint_angle_);
    }

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    seg.segment(*inliers, *coefficients);
    NODELET_INFO("input points xyz: %lu, normal: %lu",
                 cloud_xyz->points.size(), cloud_normal->points.size());

    // compute center point and cylinder height
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud_xyz);
    extract.setIndices(inliers);
    extract.filter(*cloud_cylinder);


    if (inliers->indices.size() > min_size_) {

      // publish inliers
      PCLIndicesMsg ros_inliers;
      ros_inliers.indices = inliers->indices;
      ros_inliers.header = msg->header;
      pub_inliers_.publish(ros_inliers);

      // compute pose
      Eigen::Vector3f pos(coefficients->values[0],
                          coefficients->values[1],
                          coefficients->values[2]);
      Eigen::Vector3f dir(coefficients->values[3],
                          coefficients->values[4],
                          coefficients->values[5]);
      // Cylinder always should direct to origin of pointcloud
      if (dir.dot(Eigen::Vector3f::UnitZ()) < 0) {
        dir = - dir;
      }

      Eigen::Affine3f pose = Eigen::Affine3f::Identity();
      Eigen::Quaternionf rot;
      rot.setFromTwoVectors(Eigen::Vector3f::UnitZ(), dir);
      pose = pose * Eigen::Translation3f(pos) * Eigen::AngleAxisf(rot);

      // publish coefficients
      PCLModelCoefficientMsg ros_coefficients;
      ros_coefficients.values = coefficients->values;
      ros_coefficients.header = msg->header;
      pub_coefficients_.publish(ros_coefficients);

      // publish cylinder
      jsk_recognition_msgs::Cylinder cylinder_msg;
      cylinder_msg.header = msg->header;
      tf::poseEigenToMsg(pose, cylinder_msg.pose);
      cylinder_msg.radius = coefficients->values[6];
      pub_cylinder_.publish(cylinder_msg);

      // publish cylinders
      jsk_recognition_msgs::CylinderArray cylinder_array_msg;
      cylinder_array_msg.header = msg->header;
      cylinder_array_msg.cylinders.push_back(cylinder_msg);
      pub_cylinder_array_.publish(cylinder_array_msg);

      // publish pose stamped
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header = cylinder_msg.header;
      pose_stamped.pose = cylinder_msg.pose;
      pub_pose_stamped_.publish(pose_stamped);
      pub_cylinder_array_with_failure_.publish(cylinder_array_msg);
      pub_cylinder_with_failure_.publish(cylinder_msg);
    }
    else {
      jsk_recognition_msgs::Cylinder cylinder_msg;
      cylinder_msg.header = msg->header;
      cylinder_msg.failure = true;
      pub_cylinder_with_failure_.publish(cylinder_msg);
      jsk_recognition_msgs::CylinderArray cylinder_array_msg;
      cylinder_array_msg.header = msg->header;
      cylinder_array_msg.cylinders.push_back(cylinder_msg);
      pub_cylinder_array_with_failure_.publish(cylinder_array_msg);
      NODELET_INFO("failed to find cylinder");
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::CylinderFinder, nodelet::Nodelet);
