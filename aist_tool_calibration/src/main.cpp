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
	auto	image = cv_bridge::toCvCopy(
			    *image_msg,
			    sensor_msgs::image_encodings::TYPE_8UC1)->image;
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
