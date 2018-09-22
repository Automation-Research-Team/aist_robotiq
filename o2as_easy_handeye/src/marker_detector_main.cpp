/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/
/**
* @file simple_single.cpp
* @author Bence Magyar
* @date June 2012
* @version 0.1
* @brief ROS version of the example named "simple" in the Aruco software package.
*/

#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <aruco_ros/ArucoThresholdConfig.h>
#include <aruco_ros/aruco_ros_utils.h>

namespace aruco_ros
{
class ArucoSimple
{
  private:
    ros::NodeHandle			_nh;

  // camera_info stuff
    ros::Subscriber			_cam_info_sub;
    aruco::CameraParameters		_camParam;
    bool				_useRectifiedImages;
    tf::StampedTransform		_rightToLeft;

  // transformation stuff
    const tf::TransformListener		_tfListener;
    tf::TransformBroadcaster		_tfBroadcaster;
    std::string				_marker_frame;
    std::string				_camera_frame;
    std::string				_reference_frame;

  // input/output image stuff
    image_transport::ImageTransport	_it;
    const image_transport::Subscriber	_image_sub;
    const image_transport::Publisher	_image_pub;
    const image_transport::Publisher	_debug_pub;

  // input pointcloud stuff
  //const ros::Subscriber		_pointcloud_sub;

    const ros::Publisher		_pose_pub;
    const ros::Publisher		_transform_pub; 
    const ros::Publisher		_position_pub;
    const ros::Publisher		_marker_pub;
    const ros::Publisher		_pixel_pub;

    dynamic_reconfigure::Server<ArucoThresholdConfig>
					_dyn_rec_server;

    aruco::MarkerDetector		_mDetector;
    double				_marker_size;
    int					_marker_id;

  public:
    ArucoSimple()
	:_nh("~"),
	 _cam_info_sub(_nh.subscribe("/camera_info", 1,
				     &ArucoSimple::cam_info_callback, this)),
	 _it(_nh),
	 _image_sub(_it.subscribe("/image", 1,
				  &ArucoSimple::image_callback, this)),
	 _image_pub(_it.advertise("result", 1)),
       /*
	 _pointcloud_sub(_nh.subscribe("/pointclooud", 1,
				       &ArucoSimple::pointcloud_callback,
				       this)),
       */
	 _debug_pub(_it.advertise("debug",  1)),
	 _pose_pub(_nh.advertise<geometry_msgs::PoseStamped>("pose", 100)),
	 _transform_pub(_nh.advertise<geometry_msgs::TransformStamped>(
			    "transform", 100)),
	 _position_pub(_nh.advertise<geometry_msgs::Vector3Stamped>(
			   "position", 100)),
	 _marker_pub(_nh.advertise<visualization_msgs::Marker>("marker", 10)),
	 _pixel_pub(_nh.advertise<geometry_msgs::PointStamped>("pixel", 10))
    {
	using	aruco::MarkerDetector;
	
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
	ROS_INFO_STREAM("Threshold method: "
			<< _mDetector.getThresholdMethod());
	double	th1, th2;
	_mDetector.getThresholdParams(th1, th2);
	ROS_INFO_STREAM("Threshold method: "
			<< " th1: " << th1 << " th2: " << th2);
	float	mins, maxs;
	_mDetector.getMinMaxSize(mins, maxs);
	ROS_INFO_STREAM("Marker size min: " << mins << "  max: " << maxs);
	ROS_INFO_STREAM("Desired speed: " << _mDetector.getDesiredSpeed());
    
	_nh.param("marker_size",	_marker_size,	     0.05);
	_nh.param("marker_id",		_marker_id,	     300);
	_nh.param("reference_frame",	_reference_frame,    std::string(""));
	_nh.param("camera_frame",	_camera_frame,	     std::string(""));
	_nh.param("marker_frame",	_marker_frame,	     std::string(""));
	_nh.param("image_is_rectified", _useRectifiedImages, true);

	ROS_ASSERT(_camera_frame != "" && _marker_frame != "");

	if (_reference_frame.empty())
	    _reference_frame = _camera_frame;

	ROS_INFO("Aruco node started with marker size of %f m and marker id to track: %d",
		 _marker_size, _marker_id);
	ROS_INFO("Aruco node will publish pose to TF with %s as parent and %s as child.",
		 _reference_frame.c_str(), _marker_frame.c_str());

	_dyn_rec_server.setCallback(boost::bind(&ArucoSimple::reconf_callback,
						this, _1, _2));
    }

    bool
    getTransform(const std::string& refFrame, const std::string& childFrame,
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
					ros::Time(0),//get latest available
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


    void
    image_callback(const sensor_msgs::ImageConstPtr& msg)
    {
	if ((_image_pub.getNumSubscribers()	== 0) &&
	    (_debug_pub.getNumSubscribers()	== 0) &&
	    (_pose_pub.getNumSubscribers()	== 0) &&
	    (_transform_pub.getNumSubscribers()	== 0) &&
	    (_position_pub.getNumSubscribers()	== 0) &&
	    (_marker_pub.getNumSubscribers()	== 0) &&
	    (_pixel_pub.getNumSubscribers()	== 0))
	{
	    ROS_DEBUG("No subscribers, not looking for aruco markers");
	    return;
	}

	if (!_camParam.isValid())
	    return;

	try
	{
	    const auto	curr_stamp = ros::Time::now();
	    auto	inImage = cv_bridge::toCvCopy(
				      msg, sensor_msgs::image_encodings::RGB8)
				->image;

	  //detection results will go into "markers"
	    std::vector<aruco::Marker>	markers;
	    _mDetector.detect(inImage,
			      markers, _camParam, _marker_size, false);
	    
	  //for each marker, draw info and its boundaries in the image
	    for (const auto& marker : markers)
	    {
	      // only publishing the selected marker
		if (marker.id == _marker_id)
		{
		    tf::Transform	transform = arucoMarker2Tf(marker);

		    tf::StampedTransform cameraToReference;
		    cameraToReference.setIdentity();
		    if (_reference_frame != _camera_frame)
			getTransform(_reference_frame, _camera_frame,
				     cameraToReference);

		    transform = static_cast<tf::Transform>(cameraToReference) 
			      * static_cast<tf::Transform>(_rightToLeft) 
			      * transform;

		    tf::StampedTransform
			stampedTransform(transform, curr_stamp,
					 _reference_frame, _marker_frame);
		    _tfBroadcaster.sendTransform(stampedTransform);

		    geometry_msgs::PoseStamped		poseMsg;
		    tf::poseTFToMsg(transform, poseMsg.pose);
		    poseMsg.header.frame_id = _reference_frame;
		    poseMsg.header.stamp    = curr_stamp;
		    _pose_pub.publish(poseMsg);

		    geometry_msgs::TransformStamped	transformMsg;
		    tf::transformStampedTFToMsg(stampedTransform,
						transformMsg);
		    _transform_pub.publish(transformMsg);

		    geometry_msgs::Vector3Stamped	positionMsg;
		    positionMsg.header = transformMsg.header;
		    positionMsg.vector = transformMsg.transform.translation;
		    _position_pub.publish(positionMsg);

		    geometry_msgs::PointStamped		pixelMsg;
		    pixelMsg.header  = transformMsg.header;
		    pixelMsg.point.x = marker.getCenter().x;
		    pixelMsg.point.y = marker.getCenter().y;
		    pixelMsg.point.z = 0;
		    _pixel_pub.publish(pixelMsg);

		  //Publish rviz marker representing the ArUco marker patch
		    visualization_msgs::Marker		visMarker;
		    visMarker.header   = transformMsg.header;
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
		    _marker_pub.publish(visMarker);
		}

	      // but drawing all the detected markers
		marker.draw(inImage, cv::Scalar(0,0,255), 2);
	    }

	  //draw a 3d cube in each marker if there is 3d info
	    if (_marker_size != -1)
	    {
		for (auto& marker : markers)
		    aruco::CvDrawingUtils::draw3dAxis(inImage,
						      marker, _camParam);
	    }

	    if (_image_pub.getNumSubscribers() > 0)
	    {
	      //show input with augmented information
		cv_bridge::CvImage	out_msg;
		out_msg.header.stamp = curr_stamp;
		out_msg.encoding     = sensor_msgs::image_encodings::RGB8;
		out_msg.image	     = inImage;
		_image_pub.publish(out_msg.toImageMsg());
	    }

	    if (_debug_pub.getNumSubscribers() > 0)
	    {
	      //show also the internal image
	      //resulting from the threshold operation
		cv_bridge::CvImage	debug_msg;
		debug_msg.header.stamp = curr_stamp;
		debug_msg.encoding     = sensor_msgs::image_encodings::MONO8;
		debug_msg.image	       = _mDetector.getThresholdedImage();
		_debug_pub.publish(debug_msg.toImageMsg());
	    }
	}
	catch (const cv_bridge::Exception& e)
	{
	    ROS_ERROR("cv_bridge exception: %s", e.what());
	    return;
	}
    }

  // wait for one camerainfo, then shut down that subscriber
    void
    cam_info_callback(const sensor_msgs::CameraInfo& msg)
    {
	_camParam = rosCameraInfo2ArucoCamParams(msg, _useRectifiedImages);

      // handle cartesian offset between stereo pairs
      // see the sensor_msgs/CamaraInfo documentation for details
	_rightToLeft.setIdentity();
	_rightToLeft.setOrigin(tf::Vector3(-msg.P[3]/msg.P[0],
					   -msg.P[7]/msg.P[5],
					   0.0));
	_cam_info_sub.shutdown();
    }

    void
    reconf_callback(ArucoThresholdConfig& config, uint32_t level)
    {
	_mDetector.setThresholdParams(config.param1, config.param2);

	if (config.normalizeImage)
	    ROS_WARN("normalizeImageIllumination is unimplemented!");
    }
};
}	// namespace aruco_ros

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_simple");

    aruco_ros::ArucoSimple	node;
    ros::spin();

    return 0;
}
