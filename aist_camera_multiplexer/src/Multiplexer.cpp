/*!
 *  \file	Multiplexer.cpp
 *  \author	Toshio UESHIBA
 *  \brief	ROS node for applying filters to depth images
 */
#include "Multiplexer.h"

namespace aist_camera_multiplexer
{
/************************************************************************
*  static functions							*
************************************************************************/

/************************************************************************
*  class Multiplexer::SyncedSubscribers					*
************************************************************************/
Multiplexer::SyncedSubscribers::SyncedSubscribers(Multiplexer& multiplexer,
						  size_t n)
    :_camera_info_sub(multiplexer._nh, "/camera_info" + std::to_string(n), 1),
     _image_sub(multiplexer._nh, "/image" + std::to_string(n), 1),
     _depth_sub(multiplexer._nh, "/depth" + std::to_string(n), 1),
     _sync(sync_policy_t(10), _camera_info_sub, _image_sub, _depth_sub)
{
    _sync.registerCallback(boost::bind(&Multiplexer::synced_images_cb,
				       multiplexer, _1, _2, _3, n));
}

/************************************************************************
*  class Multiplexer							*
************************************************************************/
Multiplexer::Multiplexer(const ros::NodeHandle& nh)
    :_nh(nh),
     _selectCamera_srv(_nh.advertiseService("select_camera",
					    &select_camera_cb, this)),
     _subscribers(),
     _camera_number(0),
     _it(_nh),
     _image_pub(_it.advertise("image", 1)),
     _depth_pub(_it.advertise("depth", 1)),
     _camera_info_pub(_nh.advertise<camera_info_t>("camera_info", 1))
{
    int	ncameras;
    _nh.param("number_of_cameras", ncameras, 1);
    for (size_t n = 0; n < ncameras; ++n)
	_subscribers.emplace_back(new SyncedSubscribers(*this, n));
}

void
Multiplexer::run()
{
    ros::spin();
}

bool
Multiplexer::select_camera_cb(SelectCamera::Request&  req,
			      SelectCamera::Response& res)
{
    if (0 <= req.camera_number && req.camera_number < _subscribers.size())
    {
	_camera_number = req.camera_number;
	res.success = true;

	ROS_INFO_STREAM("(Multiplexer) select camera number["
			<< _camera_number << ']');
    }
    else
    {
	res.success = false;

	ROS_ERROR_STREAM("(Multiplexer) camera number["
			 << req.camera_number << "] is out of range");
    }

    return true;
}

void
Multiplexer::synced_images_cb(const camera_info_cp& camera_info,
			      const image_cp& image,
			      const image_cp& depth,
			      size_t camera_number) const
{
    if (camera_number == _camera_number)
    {
	_camera_info_pub.publish(camera_info);
	_image_pub.publish(image);
	_depth_pub.publish(depth);
    }
}

}	// namespace aist_camera_multiplexer
