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
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <jsk_perception/draw_labeled_rect.h>

namespace enc = sensor_msgs::image_encodings;

namespace jsk_perception
{
  void DrawLabeledRect::onInit()
  {
    ConnectionBasedNodelet::onInit();

    pnh_->param("approximate_sync", approximate_sync_, false);
    pnh_->param("queue_size", queue_size_, 100);
    pnh_->param("labels", labels_, std::vector<std::string>());

    if (labels_.size() > 0) {
      label_num_ = labels_.size();
    } else {
      label_num_ = 100;
    }

    pub_image_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);

    onInitPostProcess();
  }

  void DrawLabeledRect::subscribe()
  {
    sub_image_.subscribe(*pnh_,   "input", 1);
    sub_rects_.subscribe(*pnh_, "input/rect_array", 1);
    if (approximate_sync_) {
      async_rects_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicyRects> >(queue_size_);
      async_rects_->connectInput(sub_image_, sub_rects_);
      async_rects_->registerCallback(boost::bind(&DrawLabeledRect::callbackRects, this, _1, _2));
    } else {
      sync_rects_ = boost::make_shared<message_filters::Synchronizer<ExactSyncPolicyRects> >(queue_size_);
      sync_rects_->connectInput(sub_image_, sub_rects_);
      sync_rects_->registerCallback(boost::bind(&DrawLabeledRect::callbackRects, this, _1, _2));
    }
  }

  void DrawLabeledRect::unsubscribe()
  {
    sub_image_.unsubscribe();
    sub_rects_.unsubscribe();
  }

  void DrawLabeledRect::getColorString(const jsk_recognition_msgs::Label &label, cv::Scalar& color)
  {
    static const float colors[6][3] = { {1,0,1}, {0,0,1},{0,1,1},{0,1,0},{1,1,0},{1,0,0} };

    int idx = label.index;
    if (!labels_.empty()) {
      std::vector<std::string>::iterator it = std::find(labels_.begin(), labels_.end(), label.name);
      if (it - labels_.begin() < labels_.size())
        idx = it - labels_.begin();
    }

    float ratio = ((float)(idx * 123457 % label_num_) / label_num_) * 5;
    int i = std::floor(ratio);
    int j = std::ceil(ratio);
    ratio -= i;
    for (int c = 0; c < 3; ++c)
      color[c] = (int)(((1-ratio) * colors[i][c] + ratio * colors[j][c]) * 255);
  }

  void DrawLabeledRect::draw(cv::Mat& mat, const jsk_recognition_msgs::LabeledRect &msg)
  {
    static const double font_size = 1.5;
    static const int font_thickness = 5;

    cv::Scalar color;
    getColorString(msg.label, color);

    cv::Rect rect(msg.rect.x, msg.rect.y, msg.rect.width, msg.rect.height);

    cv::rectangle(mat, rect, color, 2, CV_AA);
    if (!msg.label.name.empty()) {
      int baseline;
      cv::Size label_size = cv::getTextSize(msg.label.name, cv::FONT_HERSHEY_SIMPLEX,
                                            font_size, font_thickness, &baseline);
      cv::rectangle(mat,
                    cv::Rect(rect.x, rect.y-label_size.height-1,
                             label_size.width+1, label_size.height+1),
                    color, -1, CV_AA);
      cv::putText(mat, msg.label.name, cv::Point(rect.x, rect.y),
                  cv::FONT_HERSHEY_SIMPLEX,
                  font_size, cv::Scalar(255,255,255), font_thickness, CV_AA);
    }
  }

  void DrawLabeledRect::callbackRects(
    const sensor_msgs::Image::ConstPtr &img_msg,
    const jsk_recognition_msgs::LabeledRectArray::ConstPtr &rects)
  {
    cv_bridge::CvImage::Ptr img;
    try {
      img = cv_bridge::toCvCopy(img_msg, enc::BGR8);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("Failed to convert image to cv::Mat");
      return;
    }
    for (int i = 0; i < rects->rects.size(); ++i){
      draw(img->image, rects->rects[i]);
    }
    pub_image_.publish(*img->toImageMsg());
  }
} // namespace jsk_perception

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_perception::DrawLabeledRect, nodelet::Nodelet)