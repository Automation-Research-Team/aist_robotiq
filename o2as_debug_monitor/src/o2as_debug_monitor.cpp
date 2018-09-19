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

  class Monitor
  {
    public:
      Monitor(ros::NodeHandle &nh)
      {
        image_transport::ImageTransport it_(nh);
        sub_rs_ = it_.subscribe("/b_bot_camera/rgb/image_raw", 1,
                                &Monitor::realsenseCallback, this);
      }

    private:
      image_transport::Subscriber sub_rs_;

      cv::Mat monitor_;
      cv::Mat realsense_img_;

      void realsenseCallback(const sensor_msgs::ImageConstPtr &msg)
      {
        // Show window at the first time
        if (!cvGetWindowHandle("Monitor")) {
          cv::namedWindow("Monitor", cv::WINDOW_NORMAL);
          cv::moveWindow("Monitor", 128, 128);
        }

        // Create image buffer
        monitor_ = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255, 0, 0));

//        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
//          msg, sensor_msgs::image_encodings::BGR8
//        );



        // Copy realsense image to the image buffer
        realsense_img_ = convert2OpenCV(
          *msg, sensor_msgs::image_encodings::RGB8) -> image;
        cv::Size size = cv::Size(320, 240);
        cv::Rect rect = cv::Rect(0, 0, 320, 240);
        cv::Mat resized;
        cv::resize(realsense_img_, resized, size, 0, 0, cv::INTER_CUBIC);

        //realsense_img_.copyTo(monitor_(rect));
        resized.copyTo(monitor_(rect));

        // Copy the image buffer in the window
        cv::imshow("Monitor", monitor_);
        cv::waitKey(3);
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
