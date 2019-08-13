/*!
 *  \file	main.cpp
 *  \brief
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include "TU/Image++.h"

namespace TU
{
/************************************************************************
*  static functions							*
************************************************************************/
template <class T> Image<T>
msgToImage(const sensor_msgs::ImageConstPtr& msg)
{
    using namespace	sensor_msgs;

    TU::Image<T>	image(msg->width, msg->height);
    auto		p = msg->data.data();

    if (msg->encoding == image_encodings::RGB8)
	for (auto&& line : image)
	{
	    constexpr auto
		N = iterator_value<pixel_iterator<const RGB*> >::npixels;

	    std::copy_n(make_pixel_iterator(
			    reinterpret_cast<const RGB*>(p)),
			line.size()/N, make_pixel_iterator(line.begin()));
	    p += msg->step;
	}
    else if (msg->encoding == image_encodings::BGR8)
	for (auto&& line : image)
	{
	    constexpr auto
		N = iterator_value<pixel_iterator<const BGR*> >::npixels;

	    std::copy_n(make_pixel_iterator(
			    reinterpret_cast<const BGR*>(p)),
			line.size()/N, make_pixel_iterator(line.begin()));
	    p += msg->step;
	}
    else if (msg->encoding == image_encodings::BGRA8)
	for (auto&& line : image)
	{
	    constexpr auto
		N = iterator_value<pixel_iterator<const BGRA*> >::npixels;

	    std::copy_n(make_pixel_iterator(
			    reinterpret_cast<const BGRA*>(p)),
			line.size()/N, make_pixel_iterator(line.begin()));
	    p += msg->step;
	}
    else if (msg->encoding == image_encodings::RGBA8)
	for (auto&& line : image)
	{
	    constexpr auto
		N = iterator_value<pixel_iterator<const RGBA*> >::npixels;

	    std::copy_n(make_pixel_iterator(
			    reinterpret_cast<const RGBA*>(p)),
			line.size()/N, make_pixel_iterator(line.begin()));
	    p += msg->step;
	}
    else if (msg->encoding == image_encodings::YUV422)
	for (auto&& line : image)
	{
	    constexpr auto
		N = iterator_value<pixel_iterator<const YUV422*> >::npixels;

	    std::copy_n(make_pixel_iterator(
			    reinterpret_cast<const YUV422*>(p)),
			line.size()/N, make_pixel_iterator(line.begin()));
	    p += msg->step;
	}
    else
	for (auto&& line : image)
	{
	    std::copy_n(make_pixel_iterator(p), line.size(),
			make_pixel_iterator(line.begin()));
	    p += msg->step;
	}

    return image;
}

static sensor_msgs::ImagePtr
imageToMsg(const Image<uint8_t>& image, const std_msgs::Header& header)
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
    void	callback(const image_p&	image_msg,
			 const camera_info_p&	camera_info_msg)	;

  private:
    ros::NodeHandle				_nh;

  // transformation stuff
    const tf::TransformListener			_listener;

  // image stuff
    image_transport::ImageTransport		_it;
    image_transport::CameraSubscriber		_sub;
    const image_transport::CameraPublisher	_pub;

    std::string					_reference_frame;
    std::string					_effector_frame;
};

Calibrator::Calibrator()
    :_nh("~"),
     _it(_nh),
     _listener(),
     _sub(_it.subscribeCamera("/image", 1, &Calibrator::callback, this)),
     _pub(_it.advertiseCamera("image", 1)),
     _reference_frame("workspace_center"),
     _effector_frame("")
{
    _nh.param<std::string>("reference_frame", _reference_frame,
			   "workspace_center");
    _nh.param<std::string>("effector_frame", _effector_frame,
			   "a_bot_gripper_tip_link");
}

void
Calibrator::callback(const image_p& image_msg,
		     const camera_info_p& camera_info_msg)
{
    try
    {
	_listener.waitForTransform(_reference_frame, _effector_frame,
				   image_msg->header.stamp, ros::Duration(10));
	transform_t	T;
	_listener.lookupTransform(_reference_frame, _effector_frame,
	 			  image_msg->header.stamp, T);
	ROS_INFO_STREAM("image stamp = " << image_msg->header.stamp);
	auto	image = msgToImage<uint8_t>(image_msg);
	auto	msg   = imageToMsg(image, camera_info_msg->header);

	_pub.publish(msg, camera_info_msg);
    }
    catch (const std::runtime_error& e)
    {
	ROS_WARN_STREAM(e.what());
    }
}

}	// namespace TU

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "aist_tool_calibration");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
				   ros::console::levels::Debug);

    try
    {
	TU::Calibrator	node;
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
