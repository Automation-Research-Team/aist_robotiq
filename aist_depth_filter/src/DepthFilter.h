/*!
 *  \file	DepthFilter.h
 *  \author	Toshio UESHIBA
 *  \brief	Thin wraper of Photoneo Localization SDK
 */
#ifndef DEPTHFILTER_H
#define DEPTHFILTER_H

#include <iostream>
#include <cstdint>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

namespace aist_depth_filter
{
/************************************************************************
*  class DepthFilter							*
************************************************************************/
class DepthFilter
{
  private:
    using value_type	 = float;
    using camera_info_t	 = sensor_msgs::CameraInfo;
    using camera_info_cp = sensor_msgs::CameraInfoConstPtr;
    using image_t	 = sensor_msgs::Image;
    using image_cp	 = sensor_msgs::ImageConstPtr;
    using image_p	 = sensor_msgs::ImagePtr;
    using sync_policy_t	 = message_filters::sync_policies::
			       ApproximateTime<camera_info_t, image_t, image_t>;

  public:
		DepthFilter(const std::string& name)			;

    void	run()							;

  private:
    bool	saveBG_cb(std_srvs::Trigger::Request&  req,
			  std_srvs::Trigger::Response& res)		;
    bool	saveAsOply_cb(std_srvs::Trigger::Request&  req,
			      std_srvs::Trigger::Response& res)		;
    void	filter_cb(const camera_info_cp& camera_info,
			  const image_cp& image, const image_cp& depth)	;

    template <class T>
    void	filter(const camera_info_t& camera_info,
		       const image_t& image, image_t& depth)		;
    template <class T>
    void	saveBG(image_t& depth)				  const	;
    template <class T>
    void	removeBG(image_t& depth, const image_t& bg_depth) const	;
    template <class T>
    void	depth_clip(image_t& depth)			  const	;
    template <class T>
    void	roi(image_t& depth)				  const	;
    template <class T>
    void	scale(image_t& depth)				  const	;
    template <class T>
    void	saveAsOply(const camera_info_t& camera_info,
			   const image_t& image,
			   const image_t& depth)		  const	;

  private:
    ros::NodeHandle					_nh;

    const ros::ServiceServer				_saveBG_srv;
    const ros::ServiceServer				_saveAsOply_srv;

    message_filters::Subscriber<camera_info_t>		_camera_info_sub;
    message_filters::Subscriber<image_t>		_image_sub;
    message_filters::Subscriber<image_t>		_depth_sub;
    message_filters::Synchronizer<sync_policy_t>	_sync;

    image_transport::ImageTransport			_it;
    const image_transport::Publisher			_image_pub;
    const image_transport::Publisher			_depth_pub;
    const ros::Publisher				_camera_info_pub;

    ddynamic_reconfigure::DDynamicReconfigure		_ddr;

    camera_info_cp					_camera_info;
    image_cp						_image;
    image_cp						_depth;
    image_p						_filtered_depth;
    image_cp						_bg_depth;

  // Remove background.
    double						_threshBG;
    std::string						_fileBG;

  // Clip outside of [_near, _far].
    bool						_depth_clip;
    double						_near;
    double						_far;

  // Mask outside of ROI.
    bool						_roi;
    int							_top;
    int							_bottom;
    int							_left;
    int							_right;

  // Scaling of depth values.
    double						_scale;
};

}	// namespace aist_photoneo_localization
#endif	// DEPTHFILTER_H
