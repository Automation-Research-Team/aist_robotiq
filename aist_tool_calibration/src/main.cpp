/**
* @file simple_single.cpp
* @author Bence Magyar
* @date June 2012
* @version 0.1
* @brief ROS version of the example named "simple" in the Aruco software package.
*/
#include <iostream>
#include <limits>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "TU/Image++.h"

namespace aist_tool_calibration
{
/************************************************************************
*  static functions							*
************************************************************************/
static TU::Image<uint8_t>
msgToImage(const sensor_msgs::ImageConstPtr& msg)
{
    TU::Image<uint8_t>	image(msg->width, msg->height);
    auto		data = msg->data.data();
    for (auto&& row : image)
    {
	std::copy_n(data , row.size(), row.begin());
	data += msg->step;
    }

    return image;
}

static sensor_msgs::ImagePtr
imageToMsg(const TU::Image<uint8_t>& image, const std_msgs::Header& header)
{
    using namespace	sensor_msgs;

    sensor_msgs::ImagePtr	msg(new sensor_msgs::Image());
    msg->header		= header;
    msg->encoding	= image_encodings::MONO8;
    msg->is_bigendian	= 0;
    msg->height		= image.height();
    msg->width		= image.width();
    msg->step		= msg->width
			*  image_encodings::numChannels(msg->encoding)
			* (image_encodings::bitDepth(msg->encoding)/8);
    msg->data.resize(msg->step * msg->height);

    auto	data = msg->data.data();
    for (const auto& row : image)
    {
	std::copy_n(row.begin(), row.size(), data);
	data += msg->step;
    }

    return msg;
}

/************************************************************************
*  class Calibrator							*
************************************************************************/
class Calibrator
{
  private:
    using camera_info_t	= sensor_msgs::CameraInfo;
    using camera_info_p	= sensor_msgs::CameraInfoConstPtr;
    using image_t	= sensor_msgs::Image;
    using image_p	= sensor_msgs::ImageConstPtr;
    using transform_t	= tf::StampedTransform;

  public:
    Calibrator();

  private:
    void	camera_cb(const image_p&	image_msg,
			  const camera_info_p&	camera_info_msg)	;

  private:
    ros::NodeHandle				_nh;

  // transformation stuff
    const tf::TransformListener			_listener;

  // image stuff
    image_transport::ImageTransport		_it;
    image_transport::CameraSubscriber		_camera_sub;
    const image_transport::CameraPublisher	_camera_pub;

    std::string					_reference_frame;
    std::string					_effector_frame;
};

Calibrator::Calibrator()
    :_nh("~"),
     _it(_nh),
     _listener(),
     _camera_sub(_it.subscribeCamera("image", 1, &Calibrator::camera_cb, this)),
     _camera_pub(),
     _reference_frame("workspace_center"),
     _effector_frame("")
{
    _nh.param<std::string>("reference_frame", _reference_frame,
			   "workspace_center");
    _nh.param<std::string>("effector_frame", _effector_frame,
			   "a_bot_gripper_tip_link");
}

void
Calibrator::camera_cb(const image_p& image_msg,
		      const camera_info_p& camera_info_msg)
{
    try
    {
	// transform_t	T;
	// _listener.lookupTransform(_reference_frame, _effector_frame,
	// 			  image_msg->header.stamp, T);
	ROS_INFO_STREAM("image stamp = " << image_msg->header.stamp);
	auto	image = msgToImage(image_msg);
	auto	msg   = imageToMsg(image, camera_info_msg->header);

	_camera_pub.publish(msg, camera_info_msg);
    }
    catch (const std::runtime_error& e)
    {
	ROS_WARN_STREAM(e.what());
    }
}

}	// namespace aist_tool_calibration

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "aist_tool_calibration");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
				   ros::console::levels::Debug);

    try
    {
	aist_tool_calibration::Calibrator	node;
	ros::spin();
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM(err.what());
	return 1;
    }
    catch (...)
    {
	ROS_ERROR_STREAM("Unknon error.");
    }

    return 0;
}
