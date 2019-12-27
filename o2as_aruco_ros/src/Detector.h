/*!
* \file		Detector.h
* \author	Toshio UESHIBA
* \brief	ARuCo marker detector using both intensity and depth images
*/
#include <iostream>
#include <limits>
#include <cstdint>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <aruco/aruco.h>

namespace o2as_aruco_ros
{
/************************************************************************
*  class Detector							*
************************************************************************/
class Detector
{
  private:
    using camera_info_t	= sensor_msgs::CameraInfo;
    using camera_info_p	= sensor_msgs::CameraInfoConstPtr;
    using image_t	= sensor_msgs::Image;
    using image_p	= sensor_msgs::ImageConstPtr;
    using sync_policy_t	= message_filters::sync_policies::
			      ApproximateTime<camera_info_t, image_t, image_t>;
    using mdetector_t	= aruco::MarkerDetector;
    using bdetector_t	= aruco::BoardDetector;

  public:
		Detector(const std::string& name)			;

    void	run()							;

  private:
    void	set_refinement_method(const std::string& method)	;
    void	set_threshold_method(const std::string& method)		;
    template <class T>
    void	set_first_param(void (mdetector_t::* get)(T&, T&) const,
				void (mdetector_t::* set)(T, T),
				T param)				;
    template <class T>
    void	set_second_param(void (mdetector_t::* get)(T&, T&) const,
				 void (mdetector_t::* set)(T, T),
				 T param)				;
    void	detect_marker_cb(const camera_info_p& camera_info_msg,
				 const image_p&	      image_msg,
				 const image_p&	      depth_msg)	;
    bool	get_transform(const std::string& refFrame,
			      const std::string& childFrame,
			      tf::StampedTransform& transform)	const	;
    tf::Transform
		get_marker_transform(const aruco::Marker& marker,
				     const image_t& depth_msg,
				     cv::Mat& image)		const	;
    template <class T> cv::Vec<T, 3>
		view_vector(T u, T v)				const	;
    template <class T> cv::Vec<T, 3>
		at(const image_t& depth_msg, int u, int v)	const	;
    template <class T> cv::Vec<T, 3>
		at(const image_t& depth_msg, T u, T v)		const	;
    void	publish_marker_info(const aruco::Marker& marker,
				    const image_t& depth_msg,
				    cv::Mat& image)			;
    void	publish_image_info(const cv::Mat& image,
				   const ros::Time& stamp)		;

  private:
    ros::NodeHandle					_nh;

  // transformation stuff
    const tf::TransformListener				_tfListener;
    tf::TransformBroadcaster				_tfBroadcaster;
    std::string						_marker_frame;
    std::string						_camera_frame;
    std::string						_reference_frame;

  // input camera_info/image stuff
    message_filters::Subscriber<camera_info_t>		_camera_info_sub;
    message_filters::Subscriber<image_t>		_image_sub;
    message_filters::Subscriber<image_t>		_depth_sub;
    message_filters::Synchronizer<sync_policy_t>	_sync;

  // camera_info stuff
    aruco::CameraParameters				_camParam;
    bool						_useRectifiedImages;
    tf::StampedTransform				_rightToLeft;

  // output image stuff
    image_transport::ImageTransport			_it;
    const image_transport::Publisher			_image_pub;
    const image_transport::Publisher			_debug_pub;

  // output cloud, pose, marker, etc. stuff
    const ros::Publisher				_pose_pub;
    const ros::Publisher				_visMarker_pub;
    const ros::Publisher				_pixel_pub;
    const ros::Publisher				_corners_pub;

    ddynamic_reconfigure::DDynamicReconfigure		_ddr;

    bdetector_t						_bDetector;
    double						_marker_size;
    int							_marker_id;

    bool						_useDepth;
    double						_planarityTolerance;
};

}	// namespace o2as_aruco_ros
