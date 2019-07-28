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
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include <aist_tool_calibration/Corners.h>

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <aruco_ros/ArucoThresholdConfig.h>
#include <aruco_ros/aruco_ros_utils.h>

#include "Plane.h"
#include "BinDescription.h"

namespace aist_tool_calibration
{
/************************************************************************
*  geometry functions							*
************************************************************************/

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
    using transform_t	= geometry_msgs::TransformStamped;

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

  // output cloud, pose, marker, etc. stuff
    const ros::Publisher			_pose_pub;
    const ros::Publisher			_visMarker_pub;

    dynamic_reconfigure::Server<ToolCalibrationConfig>
						_dyn_rec_server;
};

Calibrator::Calibrator()
    :_nh("~"),
     _it(_nh),
     _listener(),
     _camera_sub(_it, _nh, "/image", 10, &Calibrator::camera_cb),
     _camera_pub(),
     _pose_pub(     _nh.advertise<geometry_msgs::PoseStamped>("pose", 100)),
     _visMarker_pub(_nh.advertise<visualization_msgs::Marker>("marker", 10))
{
    using	aruco::MarkerDetector;

    _ts.registerCallback(&Calibrator::detect_marker_cb, this);

    std::string refinementMethod;
    _nh.param("corner_refinement", refinementMethod, std::string("LINES"));
    if (refinementMethod == "SUBPIX")
	_mDetector.setCornerRefinementMethod(MarkerDetector::SUBPIX);
    else if (refinementMethod == "HARRIS")
	_mDetector.setCornerRefinementMethod(MarkerDetector::HARRIS);
    else if (refinementMethod == "NONE")
	_mDetector.setCornerRefinementMethod(MarkerDetector::NONE);
    else
	_mDetector.setCornerRefinementMethod(MarkerDetector::LINES);

  //Print parameters of aruco marker detector:
    ROS_INFO_STREAM("Corner refinement method: "
		    << _mDetector.getCornerRefinementMethod());
    ROS_INFO_STREAM("Threshold method: " << _mDetector.getThresholdMethod());
    double	th1, th2;
    _mDetector.getThresholdParams(th1, th2);
    ROS_INFO_STREAM("Threshold method: "
		    << " th1: " << th1 << " th2: " << th2);
    float	mins, maxs;
    _mDetector.getMinMaxSize(mins, maxs);
    ROS_INFO_STREAM("Marker size min: " << mins << "  max: " << maxs);
    ROS_INFO_STREAM("Desired speed: " << _mDetector.getDesiredSpeed());

    _nh.param("marker_size",	    _marker_size,	 0.05);
    _nh.param("marker_id",	    _marker_id,		 300);
    _nh.param("reference_frame",    _reference_frame,    std::string(""));
    _nh.param("camera_frame",	    _camera_frame,	 std::string(""));
    _nh.param("marker_frame",	    _marker_frame,	 std::string(""));
    _nh.param("image_is_rectified", _useRectifiedImages, true);

    ROS_ASSERT(_camera_frame != "" && _marker_frame != "");

    if (_reference_frame.empty())
	_reference_frame = _camera_frame;

    ROS_INFO_STREAM("Aruco node started with marker size of "
		    << _marker_size << " m and marker id to track: "
		    << _marker_id);
    ROS_INFO_STREAM("Aruco node will publish pose to TF with "
		    << _reference_frame << " as parent and "
		    << _marker_frame << " as child.");

    _dyn_rec_server.setCallback(boost::bind(&Calibrator::reconf_cb, this, _1, _2));
}

void
Calibrator::camera_cb(const image_p& image_msg,
		      const camera_info_p& camera_info_msg)
{
    try
    {
	transform_t	T;
	_listener.lookupTransform(_reference_frame, _tip_frame,
				  image_msg->header.stamp, T);
	auto	inImage = cv_bridge::toCvCopy(
				*image_msg,
				sensor_msgs::image_encodings::RGB8)->image;
    }
    catch (const std::runtime_error& e)
    {
	ROS_WARN_STREAM(e.what());
    }
    catch (const cv_bridge::Exception& e)
    {
	ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    }
}

bool
Calibrator::get_transform(const std::string& refFrame,
		      const std::string& childFrame,
		      tf::StampedTransform& transform) const
{
    std::string	errMsg;
    if (!_tfListener.waitForTransform(refFrame, childFrame,
				      ros::Time(0), ros::Duration(0.5),
				      ros::Duration(0.01), &errMsg))
    {
	ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
	return false;
    }

    try
    {
	_tfListener.lookupTransform(refFrame, childFrame,
				    ros::Time(0),	//get latest available
				    transform);
    }
    catch (const tf::TransformException& e)
    {
	ROS_ERROR_STREAM("Error in lookupTransform of "
			 << childFrame << " in " << refFrame);
	return false;
    }

    return true;
}

tf::Transform
Calibrator::get_marker_transform(const aruco::Marker& marker,
			     const cloud_t& cloud_msg) const
{
    using value_t	= float;
    using iterator_t	= sensor_msgs::PointCloud2ConstIterator<value_t>;
    using rgbiterator_t	= sensor_msgs::PointCloud2Iterator<uint32_t>;
    using point_t	= cv::Vec<value_t, 3>;
    using plane_t	= o2as::Plane<value_t, 3>;

    if (marker.size() < 4)
	throw std::runtime_error("Detected not all four corners!");

  // Compute initial marker plane.
    std::vector<point_t>	points;
    for (const auto& corner : marker)
    {
	const auto	point = at<value_t>(cloud_msg, corner.x, corner.y);

	if (!std::isnan(point(2)))
	    points.push_back(point);
    }

    plane_t	plane(points.cbegin(), points.cend());

  // Compute 2D bounding box of marker.
    const int	u0 = std::floor(std::min({marker[0].x, marker[1].x,
					  marker[2].x, marker[3].x}));
    const int	v0 = std::floor(std::min({marker[0].y, marker[1].y,
					  marker[2].y, marker[3].y}));
    const int	u1 = std::ceil( std::max({marker[0].x, marker[1].x,
					  marker[2].x, marker[3].x}));
    const int	v1 = std::ceil( std::max({marker[0].y, marker[1].y,
					  marker[2].y, marker[3].y}));

  // Select 3D points close to the initial plane within the bounding box.
    points.clear();
    try
    {
	rgbiterator_t	rgb(const_cast<cloud_t&>(cloud_msg), "rgb");

	for (auto v = v0; v <= v1; ++v)
	{
	    iterator_t		xyz(cloud_msg, "x");
	    rgbiterator_t	rgb(const_cast<cloud_t&>(cloud_msg), "rgb");
	    const auto		idx = cloud_msg.width * v + u0;
	    xyz += idx;
	    rgb += idx;

	    for (auto u = u0; u <= u1; ++u)
	    {
		const auto	point = at<value_t>(cloud_msg, u, v);

		if (!std::isnan(point(2)) && plane.distance(point) <
		    _planarityTolerance)
		{
		    points.push_back(point);
		    *rgb = 0xff0000;
		}

		++xyz;
		++rgb;
	    }
	}
    }
    catch (const std::exception& err)
    {
	for (auto v = v0; v <= v1; ++v)
	{
	    iterator_t	xyz(cloud_msg, "x");
	    const auto	idx = cloud_msg.width * v + u0;
	    xyz += idx;

	    for (auto u = u0; u <= u1; ++u)
	    {
		const auto	point = at<value_t>(cloud_msg, u, v);

		if (!std::isnan(point(2)) && plane.distance(point) <
		    _planarityTolerance)
		    points.push_back(point);

		++xyz;
	    }
	}
    }

  // Fit a plane to seleceted inliers and then publish.
    plane.fit(points.cbegin(), points.cend());

    _cloud_pub.publish(cloud_msg);

  // Compute 3D coordinates of marker corners and then publish.
    point_t	corners[4];
    for (int i = 0; i < 4; ++i)
	corners[i] = plane.cross_point(view_vector(marker[i].x, marker[i].y));

    Corners	corners_msg;
    for (const auto& corner: corners)
    {
	geometry_msgs::PointStamped	pointStamped;
	pointStamped.header = cloud_msg.header;
	pointStamped.point.x = corner(0);
	pointStamped.point.y = corner(1);
	pointStamped.point.z = corner(2);

	corners_msg.corners.push_back(pointStamped);
    }
    _corners_pub.publish(corners_msg);

  // Compute p and q, i.e. marker's local x-axis and y-axis respectively.
    const auto&	n = plane.normal();
    const auto	c = (corners[2] + corners[3] - corners[0] - corners[1])
		  + (corners[1] + corners[2] -
		     corners[3] - corners[0]).cross(n);
    const auto	p = c / cv::norm(c);
    const auto	q = n.cross(p);

  // Compute marker centroid.
    const auto	centroid = 0.25*(corners[0] + corners[1] +
				 corners[2] + corners[3]);

  // Compute marker -> reference transform.
  // Post-mulriply
  //   -1 0 0
  //    0 0 1
  //    0 1 0
  // according to ROS convensions.
    // const tf::Transform		transform(tf::Matrix3x3(-p(0), n(0), q(0),
    // 							-p(1), n(1), q(1),
    // 							-p(2), n(2), q(2)),
    // 					  tf::Vector3(centroid(0),
    // 						      centroid(1),
    // 						      centroid(2)));
    const tf::Transform		transform(tf::Matrix3x3(p(0), q(0), n(0),
    							p(1), q(1), n(1),
    							p(2), q(2), n(2)),
    					  tf::Vector3(centroid(0),
    						      centroid(1),
    						      centroid(2)));
    tf::StampedTransform	cameraToReference;
    cameraToReference.setIdentity();
    if (_reference_frame != _camera_frame)
	get_transform(_reference_frame, _camera_frame, cameraToReference);

    return static_cast<tf::Transform>(cameraToReference)
	 * static_cast<tf::Transform>(_rightToLeft)
	 * transform;
}

template <class T> cv::Vec<T, 3>
Calibrator::view_vector(T u, T v) const
{
    std::vector<cv::Vec<T, 2> >	uv{{u, v}}, xy;
    cv::undistortPoints(uv, xy, _camParam.CameraMatrix, _camParam.Distorsion);

    return {xy[0][0], xy[0][1], T(1)};
}

void
Calibrator::publish_marker_info(const aruco::Marker& marker,
			    const cloud_t& cloud_msg)
{
    const auto	stamp = cloud_msg.header.stamp;
#ifdef DEBUG
    {
	tf::Transform		transform = aruco_ros::arucoMarker2Tf(marker);
	tf::StampedTransform	cameraToReference;
	tf::StampedTransform	stampedTransform(transform, stamp,
						 _reference_frame,
						 _marker_frame);
	_tfBroadcaster.sendTransform(stampedTransform);
	geometry_msgs::TransformStamped transformMsg;
	tf::transformStampedTFToMsg(stampedTransform, transformMsg);
	_transform_pub.publish(transformMsg);
    }
#endif
    const tf::StampedTransform	stampedTransform(
				    get_marker_transform(marker, cloud_msg),
				    stamp, _reference_frame, _marker_frame);
    _tfBroadcaster.sendTransform(stampedTransform);

    geometry_msgs::PoseStamped	poseMsg;
    tf::poseTFToMsg(stampedTransform, poseMsg.pose);
    poseMsg.header.frame_id = _reference_frame;
    poseMsg.header.stamp    = stamp;
    _pose_pub.publish(poseMsg);

    geometry_msgs::PointStamped	pixelMsg;
    pixelMsg.header.frame_id = _marker_frame;
    pixelMsg.header.stamp    = stamp;
    pixelMsg.point.x	     = marker.getCenter().x;
    pixelMsg.point.y	     = marker.getCenter().y;
    pixelMsg.point.z	     = 0;
    _pixel_pub.publish(pixelMsg);

  //Publish rviz marker representing the ArUco marker patch
    visualization_msgs::Marker	visMarker;
    visMarker.header   = poseMsg.header;
    visMarker.id       = 1;
    visMarker.type     = visualization_msgs::Marker::CUBE;
    visMarker.action   = visualization_msgs::Marker::ADD;
    visMarker.pose     = poseMsg.pose;
    visMarker.scale.x  = _marker_size;
    visMarker.scale.y  = 0.001;
    visMarker.scale.z  = _marker_size;
    visMarker.color.r  = 1.0;
    visMarker.color.g  = 0;
    visMarker.color.b  = 0;
    visMarker.color.a  = 1.0;
    visMarker.lifetime = ros::Duration(3.0);
    _visMarker_pub.publish(visMarker);
}

void
Calibrator::publish_image_info(const cv::Mat& image, const ros::Time& stamp)
{
    if (_image_pub.getNumSubscribers() > 0)
    {
      //show input with augmented information
	cv_bridge::CvImage	out_msg;
	out_msg.header.stamp = stamp;
	out_msg.encoding     = sensor_msgs::image_encodings::RGB8;
	out_msg.image	     = image;
	_image_pub.publish(out_msg.toImageMsg());
    }

    if (_debug_pub.getNumSubscribers() > 0)
    {
      //show also the internal image
      //resulting from the threshold operation
	cv_bridge::CvImage	debug_msg;
	debug_msg.header.stamp = stamp;
	debug_msg.encoding     = sensor_msgs::image_encodings::MONO8;
	debug_msg.image	       = _mDetector.getThresholdedImage();
	_debug_pub.publish(debug_msg.toImageMsg());
    }
}

}	// namespace aist_tool_calibration

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "o2as_single");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    try
    {
	aist_tool_calibration::Calibrator	node;
	ros::spin();

	node.print_bins(std::cout);
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
