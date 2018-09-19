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

using ImageCallback = boost::function<void(const sensor_msgs::ImageConstPtr&)>;

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
        // For debugging
        //ROS_INFO("image.width: %d", img.width);
        //ROS_INFO("image.height: %d", img.height);

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

      ROS_INFO("Image callback was invoked.");

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

  class Monitor
  {
    public:
      Monitor(ros::NodeHandle &nh)
      {
        // Initialize shared objects in this class
        image_transport::ImageTransport it_(nh);
        monitor_ = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255, 0, 0));

        // Append callback functions
        imgcbs.push_back(
          getCallbackForImage(cv::Rect(0, 0, 320, 240), monitor_)
        );
        sub_rs_ = it_.subscribe("/b_bot_camera/rgb/image_raw", 1,
                                imgcbs.back());
      }

    private:
      image_transport::Subscriber sub_rs_;

//      boost::function<void(const sensor_msgs::ImageConstPtr&)> callback;
      std::vector<ImageCallback> imgcbs;

      cv::Mat monitor_;
      cv::Mat realsense_img_;
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
