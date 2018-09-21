#include <stdio.h>
#include <iostream>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

using ImageCallback =
  boost::function<void(const sensor_msgs::ImageConstPtr&)>;
using CamInfoCallback =
  boost::function<void(const sensor_msgs::CameraInfoConstPtr&)>;

namespace o2as_debug_monitor
{
  // Adopted from team-NAIST-Panasonic ARC repository
  // https://github.com/warehouse-picking-automation-challenges/team_naist_panasonic
  cv_bridge::CvImagePtr convert2OpenCV(
    const sensor_msgs::Image &img, const std::string type
    )
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
      ROS_ERROR("Image converter, cv_bridge exception: %s", e.what());
    }
    return cv_ptr;
  }

  boost::function<void(const sensor_msgs::ImageConstPtr&)>
    getCallbackForImage(cv::Rect rect, cv::Mat& monitor_) {
    // This function returns callback function
    return [=](const sensor_msgs::ImageConstPtr &msg) {
      // Show window at the first time
      if (!cvGetWindowHandle("Monitor")) {
        cv::namedWindow("Monitor", cv::WINDOW_NORMAL);
        cv::moveWindow("Monitor", 128, 128);
      }

      // Convert sent message into cv::Mat
      cv::Mat img = convert2OpenCV(
        *msg, sensor_msgs::image_encodings::RGB8
      ) -> image;

      // Resize the image and put it to the buffer
      cv::Size size = cv::Size(rect.width, rect.height);
      cv::Mat resized;
      cv::resize(img, resized, size, 0, 0, cv::INTER_CUBIC);
      resized.copyTo(monitor_(rect));

      // Copy the image buffer in the window
        cv::imshow("Monitor", monitor_);
        cv::waitKey(3);
    };
  }

  boost::function<void(const sensor_msgs::CameraInfoConstPtr&)>
    getCallbackForCameraInfo(cv::Rect rect, cv::Mat& monitor_) {
    // This function returns callback function
    return [=](const sensor_msgs::CameraInfoConstPtr &msg) {
      // Show window at the first time
      if (!cvGetWindowHandle("Monitor")) {
        cv::namedWindow("Monitor", cv::WINDOW_NORMAL);
        cv::moveWindow("Monitor", 128, 128);
      }

      ROS_INFO("width: %d", msg->width);
//      // Convert sent message into cv::Mat
//      cv::Mat img = convert2OpenCV(
//        *msg, sensor_msgs::image_encodings::RGB8
//      ) -> image;
//
//      // Resize the image and put it to the buffer
//      cv::Size size = cv::Size(rect.width, rect.height);
//      cv::Mat resized;
//      cv::resize(img, resized, size, 0, 0, cv::INTER_CUBIC);
//      resized.copyTo(monitor_(rect));
//
//      // Copy the image buffer in the window
//        cv::imshow("Monitor", monitor_);
//        cv::waitKey(3);
    };
  }

  class Monitor
  {
    public:
      Monitor(ros::NodeHandle &nh)
      {
        // Set global parameters
        XmlRpc::XmlRpcValue global;
        nh.getParam("o2as_debug_monitor/global", global);
        setGlobalParams(global);

        // Initialize shared objects in this class
        monitor_ = cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

        // Set callback functions for image topics
        XmlRpc::XmlRpcValue image_topics;
        nh.getParam("o2as_debug_monitor/image_topics", image_topics);
        setImageCallbacks(nh, image_topics);

        // Set callback functions for CameraInfo topics
        XmlRpc::XmlRpcValue caminfo_topics;
        nh.getParam("o2as_debug_monitor/caminfo_topics", caminfo_topics);
        setCameraInfoCallbacks(nh, caminfo_topics);
      }

    private:
      std::vector<image_transport::Subscriber> i_sub_rs_;
      std::vector<ros::Subscriber> n_sub_rs_;
      std::vector<ImageCallback> imgcbs_;
      std::vector<CamInfoCallback> caminfocbs_;

      cv::Mat monitor_;

      int width;
      int height;
      int n_cols;
      int n_rows;
      int d_cols;
      int d_rows;

      cv::Rect getRect(XmlRpc::XmlRpcValue params)
      {
        int col_min = params["col"][0];
        int col_max = params["col"][1];
        int row_min = params["row"][0];
        int row_max = params["row"][1];

        int x = col_min * d_cols;
        int y = row_min * d_rows;
        int w = (col_max - col_min + 1) * d_cols - 1;
        int h = (row_max - row_min + 1) * d_rows - 1;

        ROS_INFO("%d, %d, %d, %d", x, y, w, h);

        return cv::Rect(x, y, w, h);
      }

      void setGlobalParams(XmlRpc::XmlRpcValue &params)
      {
        width = params["width"];
        height = params["height"];
        n_cols = params["n_cols"];
        n_rows = params["n_rows"];
        d_cols = width / n_cols;
        d_rows = height / n_rows;
      }

      void setImageCallbacks(ros::NodeHandle &nh,
                             XmlRpc::XmlRpcValue image_topics)
      {
        image_transport::ImageTransport it_(nh);

        for (auto it = image_topics.begin(); it != image_topics.end(); it++)
        {
          auto params = image_topics[it->first];
          std::string topic_name = params["topic_name"];
          ROS_INFO("%s", topic_name.c_str());

          // Area in the debug monitor
          cv::Rect rect = getRect(params);

          // Create, set and keep callback function
          imgcbs_.push_back(getCallbackForImage(rect, monitor_));
          i_sub_rs_.push_back(it_.subscribe(topic_name, 1, imgcbs_.back()));
        }
      }

      void setCameraInfoCallbacks(ros::NodeHandle &nh,
                                  XmlRpc::XmlRpcValue caminfo_topics)
      {
        for (auto it = caminfo_topics.begin(); it != caminfo_topics.end(); it++)
        {
          auto params = caminfo_topics[it->first];
          std::string topic_name = params["topic_name"];
          ROS_INFO("%s", topic_name.c_str());

          // Area in the debug monitor
          cv::Rect rect = getRect(params);

          // Create, set and keep callback function
          caminfocbs_.push_back(getCallbackForCameraInfo(rect, monitor_));
          n_sub_rs_.push_back(nh.subscribe(topic_name, 1, caminfocbs_.back()));
        }
      }
  };

  class O2asDebugMonitor : public nodelet::Nodelet
  {
    public:
      O2asDebugMonitor() {}
      ~O2asDebugMonitor() {}
      boost::shared_ptr<Monitor> inst_;

    private:
      virtual void onInit()
      {
        // cv::namedWindow("Monitor", cv::WINDOW_NORMAL);
        inst_.reset(new Monitor(getNodeHandle()));
      }
  };

  PLUGINLIB_EXPORT_CLASS(o2as_debug_monitor::O2asDebugMonitor, nodelet::Nodelet)
}
