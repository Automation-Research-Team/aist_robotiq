/*!
 *  \file	DepthFilter.h
 *  \author	Toshio UESHIBA
 *  \brief	Thin wraper of Photoneo Localization SDK
 */
#ifndef DEPTHFILTER_H
#define DEPTHFILTER_H

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <aist_depth_filter/FileInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include "Plane.h"

namespace aist_depth_filter
{
/************************************************************************
*  class DepthFilter							*
************************************************************************/
class DepthFilter
{
  private:
    using value_t	 = float;
    using camera_info_t	 = sensor_msgs::CameraInfo;
    using camera_info_cp = sensor_msgs::CameraInfoConstPtr;
    using image_t	 = sensor_msgs::Image;
    using image_cp	 = sensor_msgs::ImageConstPtr;
    using image_p	 = sensor_msgs::ImagePtr;
    using cloud_t	 = sensor_msgs::PointCloud2;
    using sync_policy_t	 = message_filters::sync_policies::
				ApproximateTime<camera_info_t, image_t,
						image_t, image_t>;
    using sync_policy2_t = message_filters::sync_policies::
				ApproximateTime<camera_info_t,
						image_t, image_t>;
    using file_info_t	 = aist_depth_filter::FileInfo;
    using plane_t	 = TU::Plane<value_t, 3>;

  public:
    DepthFilter(const ros::NodeHandle& nh)				;

    void	run()							;

  private:
    bool	saveBG_cb(std_srvs::Trigger::Request&  req,
			  std_srvs::Trigger::Response& res)		;
    bool	capture_cb(std_srvs::Trigger::Request&  req,
			   std_srvs::Trigger::Response& res)		;
    void	filter_with_normal_cb(const camera_info_cp& camera_info,
				      const image_cp& image,
				      const image_cp& depth,
				      const image_cp& normal)		;
    void	filter_without_normal_cb(const camera_info_cp& camera_info,
					 const image_cp& image,
					 const image_cp& depth)		;

    template <class T>
    void	filter(const camera_info_t& camera_info,
		       image_t& depth)					;
    template <class T>
    void	removeBG(image_t& depth, const image_t& depth_bg) const	;
    template <class T>
    void	z_clip(image_t& depth)				  const	;
    template <class T>
    void	computeNormal(const camera_info_t& camera_info,
			      const image_t& depth)			;
    template <class T>
    void	scale(image_t& depth)				  const	;
    plane_t	detect_bottom_plane(const camera_info_t& camera_info,
				    const image_t& image,
				    const image_t& depth,
				    float thresh)		  const	;
    void	create_subimage(const image_t& image,
				image_t& subimage)		  const	;
    void	create_colored_normal(const image_t& normal,
				      image_t& colored_normal)	  const	;
    static std::string
		open_dir()					  	;

  private:
    ros::NodeHandle					_nh;

    const ros::ServiceServer				_saveBG_srv;
    const ros::ServiceServer				_capture_srv;

    message_filters::Subscriber<camera_info_t>		_camera_info_sub;
    message_filters::Subscriber<image_t>		_image_sub;
    message_filters::Subscriber<image_t>		_depth_sub;
    message_filters::Subscriber<image_t>		_normal_sub;
    message_filters::Synchronizer<sync_policy_t>	_sync;
    message_filters::Synchronizer<sync_policy2_t>	_sync2;

    image_transport::ImageTransport			_it;
    const image_transport::Publisher			_image_pub;
    const image_transport::Publisher			_depth_pub;
    const image_transport::Publisher			_normal_pub;
    const image_transport::Publisher			_colored_normal_pub;
    const ros::Publisher				_bottom_pub;
    const ros::Publisher				_camera_info_pub;
    const ros::Publisher				_file_info_pub;

    ddynamic_reconfigure::DDynamicReconfigure		_ddr;

    camera_info_cp					_camera_info_org;
    camera_info_t					_camera_info;
    image_cp						_image_org;
    image_t						_image;
    image_cp						_depth_org;
    image_cp						_depth_bg;
    image_t						_depth;
    image_t						_normal;
    image_t						_colored_normal;

  // Remove background.
    double						_threshBG;
    std::string						_fileBG;

  // Clip outside of [_near, _far].
    double						_near;
    double						_far;

  // Mask outside of ROI.
    int							_top;
    int							_bottom;
    int							_left;
    int							_right;

  // Scaling of depth values.
    double						_scale;

  // Save as OrderPly file.
    std::string						_fileOPly;

  // Radius of window for computing normals.
    int							_window_radius;

  // Threshold of plane fitting for bottom plane detection
    double						_threshPlane;

  private:
    constexpr static double				FarMax = 4.0;
};

}	// namespace aist_photoneo_localization
#endif	// DEPTHFILTER_H
