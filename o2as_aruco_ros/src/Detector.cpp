/*!
* \file		Detector.cpp
* \author	Toshio UESHIBA
* \brief	ARuCo marker detector using both intensity and depth images
*/
#include <iostream>
#include <cstdint>

#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>

#include <aruco_ros/aruco_ros_utils.h>

#include <o2as_aruco_ros/Corners.h>
#include "Detector.h"
#include "Plane.h"

namespace o2as_aruco_ros
{
/************************************************************************
*  global functions							*
************************************************************************/
template <class T> inline T
val(const sensor_msgs::Image& image_msg, int u, int v)
{
    using namespace	sensor_msgs;

    if (image_msg.encoding == image_encodings::TYPE_16UC1)
    	return T(0.001) * *reinterpret_cast<const uint16_t*>(
    				image_msg.data.data() + v*image_msg.step
    						      + u*sizeof(uint16_t));
    else
	return *reinterpret_cast<const T*>(image_msg.data.data()
					   + v*image_msg.step + u*sizeof(T));
}

/************************************************************************
*  class Detector							*
************************************************************************/
Detector::Detector(const std::string& name)
    :_nh(name),
     _tfListener(),
     _tfBroadcaster(),
     _marker_frame(""),
     _camera_frame(""),
     _reference_frame(""),
     _camera_info_sub(_nh, "/camera_info", 1),
     _image_sub(_nh, "/image", 1),
     _depth_sub(_nh, "/depth", 1),
     _sync(sync_policy_t(10), _camera_info_sub, _image_sub, _depth_sub),
     _camParam(),
     _useRectifiedImages(true),
     _rightToLeft(),
     _it(_nh),
     _image_pub(_it.advertise("result", 1)),
     _debug_pub(_it.advertise("debug",  1)),
     _pose_pub(     _nh.advertise<geometry_msgs::PoseStamped>("pose", 100)),
     _visMarker_pub(_nh.advertise<visualization_msgs::Marker>("marker", 10)),
     _pixel_pub(    _nh.advertise<geometry_msgs::PointStamped>("pixel", 10)),
     _corners_pub(  _nh.advertise<Corners>("corners", 10)),
     _ddr(),
     _mDetector(),
     _marker_size(0.05),
     _marker_id(0),
     _useDepth(true),
     _planarityTolerance(0.001)
{
    _nh.param("marker_size",	    _marker_size,	 0.05);
    _nh.param("marker_id",	    _marker_id,		 26);
    _nh.param("marker_frame",	    _marker_frame,	 std::string(""));
    _nh.param("camera_frame",	    _camera_frame,	 std::string(""));
    _nh.param("reference_frame",    _reference_frame,    std::string(""));
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

  // Restore marker map if specified.
    std::string	mMapFile;
    _nh.param("marker_map", mMapFile, std::string(""));
    if (mMapFile != "")
	_mMap.readFromFile(mMapFile);
    
  // Setup ddynamic_reconfigure service for min/max sizes.
    float	mins, maxs;
    ROS_INFO_STREAM("Marker size min: " << mins << "  max: " << maxs);
    ROS_INFO_STREAM("Desired speed: " << _mDetector.getDesiredSpeed());

  // Set coner refinement method and setup ddynamic_reconfigure service for it.
    std::string refinementMethod;
    _nh.param("refinement_method", refinementMethod, std::string("LINES"));
    set_refinement_method(refinementMethod);

    std::map<std::string, std::string>	map_refinementMethod =
    					{
    					    {"NONE",	"NONE"},
    					    {"HARRIS",	"HARRIS"},
    					    {"SUBPIX",	"SUBPIX"},
    					    {"LINES",	"LINES"},
    					};
    _ddr.registerEnumVariable<std::string>(
    	"refinement_method", refinementMethod,
    	boost::bind(&Detector::set_refinement_method, this, _1),
    	"Corner refinement method", map_refinementMethod);
    ROS_INFO_STREAM("Corner refinement method: "
		    << _mDetector.getCornerRefinementMethod());

  // Set threshold method and setup ddynamic_reconfigure service for it.
    std::string thresholdMethod;
    _nh.param("threshold_method", thresholdMethod, std::string("ADPT_THRES"));
    set_threshold_method(thresholdMethod);

    std::map<std::string, std::string>	map_thresholdMethod =
					{
					    {"FIXED_THRES", "FIXED_THRES"},
					    {"ADPT_THRES",  "ADPT_THRES"},
					    {"CANNY",	    "CANNY"},
					};
    _ddr.registerEnumVariable<std::string>(
	"threshold_method", thresholdMethod,
	boost::bind(&Detector::set_threshold_method, this, _1),
	"Threshold method", map_thresholdMethod);
    ROS_INFO_STREAM("Threshold method: " << _mDetector.getThresholdMethod());

  // Setup ddynamic_reconfigure service for threshold values.
    double	param1, param2;
    _mDetector.getThresholdParams(param1, param2);
    _ddr.registerVariable<double>(
	"param1", param1,
	boost::bind(&Detector::set_first_param<double>, this,
		    &mdetector_t::getThresholdParams,
		    &mdetector_t::setThresholdParams, _1),
	"Size of the block used for computing threshold at its center", 1, 15);
    _ddr.registerVariable<double>(
	"param2", param2,
	boost::bind(&Detector::set_second_param<double>, this,
		    &mdetector_t::getThresholdParams,
		    &mdetector_t::setThresholdParams, _1),
	"The constant sbtracted from the mean or weighted mean", 1, 15);
    ROS_INFO_STREAM("Thresholding paramaeter values: "
		    << " param1: " << param1 << " param2: " << param2);

  // Set usage of depth and setup its ddynamic_recoconfigure service.
    _nh.param("use_depth", _useDepth, true);
    _ddr.registerVariable<bool>(
    	"use_depth", &_useDepth, "Use depth for computing marker pose");

  // Set planarity tolerance and setup its ddynamic_recoconfigure service.
    _nh.param("planarity_tolerance", _planarityTolerance, 0.001);
    _ddr.registerVariable<double>(
    	"planarity_tolerance", &_planarityTolerance,
    	"Planarity tolerance for extracting marker region(in meters)",
    	0.0005, 0.05);

  // Pulish ddynamic_reconfigure service.
    _ddr.publishServicesTopics();

  // Register callback for marker detection.
    _sync.registerCallback(&Detector::detect_marker_cb, this);
}

void
Detector::run()
{
    ros::spin();
}

void
Detector::set_refinement_method(const std::string& method)
{
    if (method == "NONE")
	_mDetector.setCornerRefinementMethod(mdetector_t::NONE);
    else if (method == "HARRIS")
	_mDetector.setCornerRefinementMethod(mdetector_t::HARRIS);
    else if (method == "SUBPIX")
	_mDetector.setCornerRefinementMethod(mdetector_t::SUBPIX);
    else
	_mDetector.setCornerRefinementMethod(mdetector_t::LINES);
}

void
Detector::set_threshold_method(const std::string& method)
{
    if (method == "FIXED_THRES")
	_mDetector.setThresholdMethod(mdetector_t::FIXED_THRES);
    else if (method == "ADPT_THRES")
	_mDetector.setThresholdMethod(mdetector_t::ADPT_THRES);
    else
	_mDetector.setThresholdMethod(mdetector_t::CANNY);
}

template <class T> inline void
Detector::set_first_param(void (mdetector_t::* get)(T&, T&) const,
			  void (mdetector_t::* set)(T, T),
			  T param)
{
    T	dummy, param2;
    (_mDetector.*get)(dummy, param2);
    (_mDetector.*set)(param, param2);
}

template <class T> inline void
Detector::set_second_param(void (mdetector_t::* get)(T&, T&) const,
			   void (mdetector_t::* set)(T, T),
			   T param)
{
    T	param1, dummy;
    (_mDetector.*get)(param1, dummy);
    (_mDetector.*set)(param1, param);
}

void
Detector::detect_marker_cb(const camera_info_p& camera_info_msg,
			   const image_p& image_msg, const image_p& depth_msg)
{
    try
    {
      // Truncate distortion coefficiens because aruco_ros accepts only
      // first four parameters.
	auto	msg_tmp = *camera_info_msg;
	msg_tmp.D.resize(4);
	std::copy_n(std::begin(camera_info_msg->D), msg_tmp.D.size(),
		    std::begin(msg_tmp.D));
	_camParam = aruco_ros::rosCameraInfo2ArucoCamParams(
			msg_tmp, _useRectifiedImages);

      // wait for one camerainfo, then shut down that subscriber
      // handle cartesian offset between stereo pairs
      // see the sensor_msgs/CamaraInfo documentation for details
	_rightToLeft.setIdentity();
	_rightToLeft.setOrigin(tf::Vector3(-camera_info_msg->P[3]/
					    camera_info_msg->P[0],
					   -camera_info_msg->P[7]/
					    camera_info_msg->P[5],
					   0.0));
	auto	image = cv_bridge::toCvCopy(*image_msg,
					    sensor_msgs::image_encodings::RGB8)
		      ->image;

      // detection results will go into "markers"
	std::vector<aruco::Marker>	markers;
	_mDetector.detect(image, markers, _camParam, _marker_size, false);

	if (markers.size() == 0)
	    throw std::runtime_error("No markers detected!");

      // for each marker, draw info and its boundaries in the image
	for (const auto& marker : markers)
	{
	    try
	    {
		if (_marker_id == 0 || marker.id == _marker_id)
		    publish_marker_info(marker, *depth_msg, image);
	    }
	    catch (const std::runtime_error& e)
	    {
		ROS_WARN_STREAM(e.what());
	    }

	  // but drawing all the detected markers
	    marker.draw(image, cv::Scalar(0, 0, 255), 2);
	}

	if (_marker_size != -1)
	    for (auto& marker : markers)
		aruco::CvDrawingUtils::draw3dAxis(image, marker, _camParam);

	publish_image_info(image, image_msg->header.stamp);
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
Detector::get_transform(const std::string& refFrame,
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
Detector::get_marker_transform(const aruco::Marker& marker,
			       const image_t& depth_msg, cv::Mat& image) const
{
    if (marker.size() < 4)
	throw std::runtime_error("Detected not all four corners!");

    tf::Transform	transform;

    if (_useDepth)
    {
	struct rgb_t	{ uint8_t r, g, b; };
	using value_t	= float;
	using point_t	= cv::Vec<value_t, 3>;
	using plane_t	= o2as::Plane<value_t, 3>;

      // Compute initial marker plane.
	std::vector<point_t>	points;
	for (const auto& corner : marker)
	{
	    const auto	point = at<value_t>(depth_msg, corner.x, corner.y);

	    if (point(2) != value_t(0))
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
	for (auto v = v0; v <= v1; ++v)
	    for (auto u = u0; u <= u1; ++u)
	    {
		const auto	point = at<value_t>(depth_msg, u, v);

		if (point(2) != value_t(0) &&
		    plane.distance(point) < _planarityTolerance)
		{
		    points.push_back(point);
		    image.at<rgb_t>(v, u).b = 0;
		}
	    }

      // Fit a plane to seleceted inliers.
	plane.fit(points.cbegin(), points.cend());

      // Compute 3D coordinates of marker corners and then publish.
	point_t	corners[4];
	for (int i = 0; i < 4; ++i)
	    corners[i] = plane.cross_point(view_vector(marker[i].x,
						       marker[i].y));

	Corners	corners_msg;
	for (const auto& corner: corners)
	{
	    geometry_msgs::PointStamped	pointStamped;
	    pointStamped.header  = depth_msg.header;
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
	transform.setBasis(tf::Matrix3x3(p(0), q(0), n(0),
					 p(1), q(1), n(1),
					 p(2), q(2), n(2)));

      // Compute marker centroid.
	const auto	centroid = 0.25*(corners[0] + corners[1] +
					 corners[2] + corners[3]);
	transform.setOrigin(tf::Vector3(centroid(0), centroid(1), centroid(2)));
    }
    else
    {
	const tf::Transform	rot(tf::Matrix3x3(-1, 0, 0,
						   0, 0, 1,
						   0, 1, 0),
				    tf::Vector3(0, 0, 0));
	transform = aruco_ros::arucoMarker2Tf(marker) * rot;
    }

    tf::StampedTransform	cameraToReference;
    cameraToReference.setIdentity();
    if (_reference_frame != _camera_frame)
	get_transform(_reference_frame, _camera_frame, cameraToReference);

    return static_cast<tf::Transform>(cameraToReference)
	 * static_cast<tf::Transform>(_rightToLeft)
	 * transform;
}

template <class T> inline cv::Vec<T, 3>
Detector::view_vector(T u, T v) const
{
    std::vector<cv::Vec<T, 2> >	uv{{u, v}}, xy;
    cv::undistortPoints(uv, xy, _camParam.CameraMatrix, _camParam.Distorsion);

    return {xy[0][0], xy[0][1], T(1)};
}

template <class T> inline cv::Vec<T, 3>
Detector::at(const image_t& depth_msg, int u, int v) const
{
    const auto	xyz = view_vector<T>(u, v);
    const auto	d   = val<T>(depth_msg, u, v);

    return {xyz[0]*d, xyz[1]*d, d};
}

template <class T> cv::Vec<T, 3>
Detector::at(const image_t& depth_msg, T u, T v) const
{
    const int	u0 = std::floor(u);
    const int	v0 = std::floor(v);
    const int	u1 = std::ceil(u);
    const int	v1 = std::ceil(v);
    for (auto vv = v0; vv <= v1; ++vv)
	for (auto uu = u0; uu <= u1; ++uu)
	{
	    const auto	xyz = at<T>(depth_msg, uu, vv);
	    if (xyz[2] != T(0))
		return xyz;
	}

    return {T(0), T(0), T(0)};
}

void
Detector::publish_marker_info(const aruco::Marker& marker,
			      const image_t& depth_msg, cv::Mat& image)
{
    const auto	stamp = depth_msg.header.stamp;
    auto	marker_frame = _marker_frame;
    if (_marker_id == 0)
	(marker_frame += '_') += std::to_string(marker.id);
    const tf::StampedTransform	stampedTransform(
				    get_marker_transform(marker, depth_msg,
							 image),
				    stamp, _reference_frame, marker_frame);
    _tfBroadcaster.sendTransform(stampedTransform);

    geometry_msgs::PoseStamped	poseMsg;
    tf::poseTFToMsg(stampedTransform, poseMsg.pose);
    poseMsg.header.frame_id = _reference_frame;
    poseMsg.header.stamp    = stamp;
    _pose_pub.publish(poseMsg);

    geometry_msgs::PointStamped	pixelMsg;
    pixelMsg.header.frame_id = marker_frame;
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
Detector::publish_image_info(const cv::Mat& image, const ros::Time& stamp)
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

}	// namespace o2as_aruco_ros
