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
#include "Similarity.h"

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
     _ddr(),
     _mDetector(),
     _mMap(),
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
    else if (_marker_id != 0)
    {
	const auto	half_size = _marker_size/2;
	markerinfo_t	mInfo(_marker_id);
	mInfo.push_back({-half_size, -half_size, 0});
	mInfo.push_back({ half_size, -half_size, 0});
	mInfo.push_back({ half_size,  half_size, 0});
	mInfo.push_back({-half_size,  half_size, 0});
	_mMap.push_back(mInfo);
	_mMap.mInfoType = markermap_t::METERS;
    }

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

      // Handle cartesian offset between stereo pairs.
      // See the sensor_msgs/CamaraInfo documentation for details.
	_rightToLeft.setIdentity();
	_rightToLeft.setOrigin(tf::Vector3(-camera_info_msg->P[3]/
					    camera_info_msg->P[0],
					   -camera_info_msg->P[7]/
					    camera_info_msg->P[5],
					   0.0));

      // Convert sensor_msgs::ImageConstPtr to CvImagePtr.
	auto	image = cv_bridge::toCvCopy(image_msg,
					    sensor_msgs::image_encodings::RGB8)
		      ->image;

      // Detect markers. Results will go into "markers"
	std::vector<aruco::Marker>	markers;
	_mDetector.detect(image, markers, _camParam, _marker_size, false);

	if (_mMap.size() > 0)
	{
	    std::vector<std::pair<point3_t, point3_t> >	pairs;

	    for (const auto& marker : markers)
	    {
		const auto	i = _mMap.getIndexOfMarkerId(marker.id);
		if (i == -1)
		    continue;

		const auto	corners = get_marker_corners(marker,
							     *depth_msg, image);
		const auto&	markerinfo = _mMap[i];

		for (size_t j = 0; j < corners.size(); ++j)
		    pairs.push_back(std::make_pair(markerinfo[j], corners[j]));
	    }

	    publish_transform(pairs.begin(), pairs.end(),
			      depth_msg->header.stamp, _marker_frame);
	}
	else
	{
	  // for each marker, draw info and its boundaries in the image
	    for (const auto& marker : markers)
	    {
		try
		{
		    const auto	corners = get_marker_corners(marker,
							     *depth_msg, image);
		    if (corners.size() < 4)
			throw std::runtime_error("Not all four corners deteced!");

		    std::vector<std::pair<point3_t, point3_t> >	pairs;
		    const auto	half_size = _marker_size/2;
		    pairs.push_back(std::make_pair(
					corners[0],
					point3_t(-half_size, -half_size, 0)));
		    pairs.push_back(std::make_pair(
					corners[1],
					point3_t( half_size, -half_size, 0)));
		    pairs.push_back(std::make_pair(
					corners[2],
					point3_t( half_size,  half_size, 0)));
		    pairs.push_back(std::make_pair(
					corners[3],
					point3_t(-half_size,  half_size, 0)));

		    publish_transform(pairs.begin(), pairs.end(),
				      depth_msg->header.stamp,
				      _marker_frame + '_'
						    + std::to_string(marker.id));
		}
		catch (const std::runtime_error& e)
		{
		    ROS_WARN_STREAM(e.what());
		}

	      // but drawing all the detected markers
		marker.draw(image, cv::Scalar(0, 0, 255), 2);
	    }
	}

	if (_marker_size != -1)
	    for (auto& marker : markers)
		aruco::CvDrawingUtils::draw3dAxis(image, marker, _camParam);

	publish_image(image, image_msg->header.stamp);
    }
    catch (const std::exception& e)
    {
	ROS_WARN_STREAM(e.what());
    }
}

template <class ITER> void
Detector::publish_transform(ITER begin, ITER end, const ros::Time& stamp,
			    const std::string& marker_frame)
{
    Similarity<float, 3>	similarity;
    const auto	residual = similarity.fit(begin, end);

    ROS_DEBUG_STREAM("Fitted similarity transformation: scale = "
		     << similarity.s() << ", residual = " << residual);

    const auto		R = similarity.R();
    const auto		t = similarity.t() * (1/similarity.s());
    tf::Transform	transform(tf::Matrix3x3(R(0, 0), R(0, 1), R(0, 2),
						R(1, 0), R(1, 1), R(1, 2),
						R(2, 0), R(2, 1), R(2, 2)),
				  tf::Vector3(t(0), t(1), t(2)));

    tf::StampedTransform	cameraToReference;
    cameraToReference.setIdentity();
    if (_reference_frame != _camera_frame)
	get_transform(_reference_frame, _camera_frame, cameraToReference);

    _tfBroadcaster.sendTransform({static_cast<tf::Transform>(cameraToReference) *
				  static_cast<tf::Transform>(_rightToLeft) *
				  transform,
				  stamp, _reference_frame, marker_frame});

    if (_pose_pub.getNumSubscribers() > 0)
    {
	geometry_msgs::PoseStamped	poseMsg;
	tf::poseTFToMsg(transform, poseMsg.pose);
	poseMsg.header.frame_id = _reference_frame;
	poseMsg.header.stamp    = stamp;
	_pose_pub.publish(poseMsg);
    }
}

void
Detector::publish_image(const cv::Mat& image, const ros::Time& stamp) const
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

std::vector<Detector::point3_t>
Detector::get_marker_corners(const aruco::Marker& marker,
			     const image_t& depth_msg, cv::Mat& image) const
{
    struct rgb_t	{ uint8_t r, g, b; };
    using value_t	= float;
    using point_t	= cv::Vec<value_t, 3>;
    using plane_t	= Plane<value_t, 3>;

    std::vector<point3_t>	corners;

    if (marker.size() < 4)
	return corners;

  // Compute initial marker plane.
    std::vector<point3_t>	points;
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
    for (const auto& corner : marker)
	corners.push_back(plane.cross_point(view_vector(corner.x, corner.y)));

    return corners;
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

}	// namespace o2as_aruco_ros
