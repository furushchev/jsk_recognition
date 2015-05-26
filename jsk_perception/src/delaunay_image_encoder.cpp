/*
 * delaunay_image_encoder.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <vector>
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>

#include <nodelet/nodelet.h>
#include <jsk_topic_tools/log_utils.h>
#include <image_transport/image_transport.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>
#include <jsk_recognition_msgs/DelaunayImage.h>


namespace enc = sensor_msgs::image_encodings;

namespace jsk_perception
{
class DelaunayImageEncoder : public nodelet::Nodelet
{
  ros::NodeHandle _nh, _lnh;
  double _rate;

  ros::Publisher _del_img_pub, _debug_img_pub;
  image_transport::Subscriber _img_sub;
  unsigned int _subscriber_count;

  jsk_recongnition_msgs::DelaunayImagePtr _del_img_ptr;

  boost::shared_ptr<image_transport::ImageTransport> _it;

  void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    process(msg, msg->header.frame_id);
  }

  void process(const sensor_msgs::ImageConstPtr &msg, const std::string input_frame_from_msg) {
    try {
      auto cv_ptr = cv_bridge::toCvShare(msg);
      cv::Mat img_mat = cv_ptr->image;
      std::vector<cv::Point2f> pts;
      int desired_pts_count = 500;

      generateEdgePoints(pts, img_mat);
      int edge_counts = pts.size();
      if (desired_pts_count - edge_counts < 100) {
        generateRandomPoints(pts, img_mat, 100);
      } else {
        generateRandomPoints(pts, img_mat, desired_pts_count - edge_counts);
      }

      // create subdiv
      cv::Subdiv2D subdiv;
      subdiv.initDelaunay(cv::Rect(0,0,img_mat.cols, img_mat.rows));
      subdiv.insert(pts);

      showDebugImage(img_mat, subdiv);
    } catch(std::Exception &e) {
      ROS_ERROR_STREAM("failed to encode delaunay image: " << e.what());
      return;
    }
  } // end of process function

  void generateEdgePoints(std::vector<cv::Point2f> &ret_vec, const cv::Mat &mat) {
    cv::Mat gray, edge;
    cv::cvtColor(mat, gray, CV_BGR2GRAY);
    cv::Canny(gray, edge, 50, 200, 3);
    for (int y = 0; y < edge.rows; ++y) {
      for (int x = 0; x < edge.cols; ++x) {
        int idx = y * edge.cols + x;
        if (edge.data[idx] > 0) {
          ret_vec.push_back(cv::Point2f(x,y));
        }
      }
    }
  }

  void generateRandomPoints(std::vector<cv::Point2f> &ret_vec, const cv::Mat &mat, const unsigned int points) {
    std::mt19937 mt;
    std::uniform_int_distribution<int> dist_x(0, mat.cols);
    std::uniform_int_distribution<int> dist_y(0, mat.rows);

    for (unsigned int i = 0; i < points; ++i) {
      ret_vec.push_back(cv::Point2f(x,y));
    }
  }

  void showDebugImage(cv::Mat &src, cv::Subdiv2D &subdiv) {
    // draw delaunay edges
    cv::Mat ret(src);
    std::vector<cv::Vec4f> edgeList;
    subdiv.getEdgeList(edgeList);
    for (auto edge = edgeList.begin(); edge != edgeList.end(); ++edge) {
      cv::line(ret, cv::Point(edge->val[0], edge->val[1]), cv::Point(edge->val[2], edge->val[3]), cv::Scalar(48,128,48));
    }

    cv::imshow("debugimage", ret);
    cv::waitKey(10);
  }

  void subscribe() {
    JSK_NODELET_DEBUG("Subscribing to image topic.");
    _img_sub = _it->subscribe("image", 3, &DelaunayImageEncoder::imageCallback, this);
  }

  void unsubscribe() {
    JSK_NODELET_DEBUG("Unsubscribing from image topic.");
    _img_sub.shutdown();
  }

  void connectCb(const ros::SingleSubscriberPublisher& ssp) {
    if (_subscriber_count++ == 0) {
      subscribe();
    }
  }

  void disconnectCb(const ros::SingleSubscriberPublisher&) {
    _subscriber_count--;
    if (_subscriber_count == 0) {
      unsubscribe();
    }
  }

public:
  void onInit() {
    _nh = getNodeHandle();
    _lnh = ros::NodeHandle("~");
    _it.reset(new image_transport::ImageTransport(_nh));
    _subscriber_count = 0;
    ros::SubscriberStatusCallback connect_cb    = boost::bind(&DelaunayImageEncoder::connectCb, this, _1);
    ros::SubscriberStatusCallback disconnect_cb = boost::bind(&DelaunayImageEncoder::disconnectCb, this, _1);
    _del_img_pub = _nh.advertise<jsk_recognition_msgs::DelaunayImage>("delaunay_image", 10, connect_cb, disconnect_cb);
    _del_img_ptr = boost::make_shared<jsk_recognition_msgs::DelaunayImage>();
    _lnh.param("rate", _rate, 3.0);
    _lnh.param("print_point_num", _print_point_num, false);
  } // end of onInit function
}; // end of SparseImageEncoder class definition
} // end of jsk_perception namespace

typedef jsk_perception::DelaunayImageEncoder DelaunayImageEncoder;
PLUGINLIB_DECLARE_CLASS (jsk_perception, DelaunayImageEncoder, DelaunayImageEncoder, nodelet::Nodelet);

