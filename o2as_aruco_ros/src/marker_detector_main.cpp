/**
* @file simple_single.cpp
* @author Bence Magyar
* @date June 2012
* @version 0.1
* @brief ROS version of the example named "simple" in the Aruco software package.
*/

#include <iostream>
#include <limits>
#include <opencv2/core.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
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
/************************************************************************
*  class ArucoSimple							*
************************************************************************/
template <class ITER> auto
fit_plane(ITER begin, ITER end)
{
    using vector_t	= typename std::iterator_traits<ITER>::value_type;
    using value_type	= typename vector_t::value_type;
    constexpr int N	= vector_t::rows;
    using matrix_t	= cv::Matx<value_type, N, N>;
    using plane_t	= cv::Vec<value_type, N+1>;

    const auto	outer = [](const auto& x)
			{
			    matrix_t	y;
			    for (int i = 0; i < N; ++i)
				for (int j = 0; j < N; ++j)
				    y(i, j) = x(i) * x(j);
			    return y;
			};
    
  // Compute centroid.
    const auto	siz = std::distance(begin, end);
    auto	centroid = vector_t::zeros();
    for (auto iter = begin; iter != end; ++iter)
	centroid += *iter;
    centroid *= value_type(1)/value_type(siz);

  // Compute moment matrix.
    auto	moments = matrix_t::zeros();
    for (auto iter = begin; iter != end; ++iter)
	moments += outer(*iter - centroid);

    matrix_t	evectors;
    vector_t	evalues;
    cv::eigen(moments, evalues, evectors);
    std::cerr << "  evalues = " << evalues << std::endl;

    plane_t	plane;
    plane(N) = 0;
    for (int i = 0; i < N; ++i)
    {
	plane(i)  = evectors(N-1, i);
	plane(N) -= plane(i) * centroid(i);
    }
    if (plane[N] < 0)
	plane *= value_type(-1);

    std::cerr << "plane = " << plane << std::endl << std::endl;
    
    return plane;
}

template <class T, int N> T
distance(const cv::Vec<T, N+1>& plane, const cv::Vec<T, N>& point)
{
    T	d = 0;
    for (int i = 0; i < N; ++i)
	d += plane(i) * point(i);

    return std::abs(d + plane(N));
}

template <class T> cv::Vec<T, 3>
at(const sensor_msgs::PointCloud2& cloud_msg, int u, int v)
{
    using iterator_t = sensor_msgs::PointCloud2ConstIterator<T>;

    iterator_t	xyz(cloud_msg, "x");
    xyz += (cloud_msg.width * v + u);

    return {xyz[0], xyz[1], xyz[2]};
	
}

template <class T> cv::Vec<T, 3>
at(const sensor_msgs::PointCloud2& cloud_msg, T u, T v)
{
    using iterator_t = sensor_msgs::PointCloud2ConstIterator<T>;

    constexpr auto	NaN = std::numeric_limits<T>::quiet_NaN();
    const int		u0 = std::floor(u);
    const int		v0 = std::floor(v);
    const int		u1 = std::ceil(u);
    const int		v1 = std::ceil(v);
    for (auto vv = v0; vv <= v1; ++vv)
    {
	iterator_t	xyz(cloud_msg, "x");
	xyz += (cloud_msg.width * vv + u0);
	
	for (auto uu = u0; uu <= u1; ++uu)
	{
	    if (!std::isnan(xyz[2]))
		return {xyz[0], xyz[1], xyz[2]};
	    ++xyz;
	}
    }
    
    return {NaN, NaN, NaN};
}

/************************************************************************
*  class ArucoSimple							*
************************************************************************/
class ArucoSimple
{
  private:
    using	image_t	= sensor_msgs::Image;
    using	image_p	= sensor_msgs::ImageConstPtr;
    using	cloud_t = sensor_msgs::PointCloud2;
    using	cloud_p	= sensor_msgs::PointCloud2ConstPtr;
    
  public:
    ArucoSimple();

  private:
    void	cam_info_callback(const sensor_msgs::CameraInfo& msg)	;
    void	reconf_callback(ArucoThresholdConfig& config,
				uint32_t level)				;
    void	image_callback(const image_p& image_msg)		;
    void	cloud_callback(const cloud_p& cloud_msg)		;
    void	detect_marker(const image_t& image_msg,
			      const cloud_t& cloud_msg)			;
    bool	get_transform(const std::string& refFrame,
			      const std::string& childFrame,
			      tf::StampedTransform& transform)	const	;
    void
		get_transform(const aruco::Marker& marker,
			      const cloud_t& cloud_msg)		const	;
    
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
    image_t				_image_msg;

  // input pointcloud stuff
    const ros::Subscriber		_cloud_sub;
    const ros::Publisher		_cloud_pub;
    cloud_t				_cloud_msg;
    
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
};
    
ArucoSimple::ArucoSimple()
    :_nh("~"),
     _cam_info_sub(_nh.subscribe("/camera_info", 1,
				 &ArucoSimple::cam_info_callback, this)),
     _it(_nh),
     _image_sub(_it.subscribe("/image", 1,
			      &ArucoSimple::image_callback, this)),
     _image_pub(_it.advertise("result", 1)),
     _cloud_sub(_nh.subscribe("/pointcloud", 1,
			      &ArucoSimple::cloud_callback, this)),
     _cloud_pub(_nh.advertise<cloud_t>("pointcloud",  1)),
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

    ROS_INFO("Aruco node started with marker size of %f m and marker id to track: %d",
	     _marker_size, _marker_id);
    ROS_INFO("Aruco node will publish pose to TF with %s as parent and %s as child.",
	     _reference_frame.c_str(), _marker_frame.c_str());

    _dyn_rec_server.setCallback(boost::bind(&ArucoSimple::reconf_callback,
					    this, _1, _2));
}

void
ArucoSimple::cam_info_callback(const sensor_msgs::CameraInfo& msg)
{
  // wait for one camerainfo, then shut down that subscriber
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
ArucoSimple::reconf_callback(ArucoThresholdConfig& config, uint32_t level)
{
    _mDetector.setThresholdParams(config.param1, config.param2);

    if (config.normalizeImage)
	ROS_WARN("normalizeImageIllumination is unimplemented!");
}

void
ArucoSimple::image_callback(const image_p& image_msg)
{
    if (image_msg->header.stamp == _cloud_msg.header.stamp)
	detect_marker(*image_msg, _cloud_msg);
    else
	_image_msg = *image_msg;
}
    
void
ArucoSimple::cloud_callback(const cloud_p& cloud_msg)
{
    if (cloud_msg->header.stamp == _image_msg.header.stamp)
	detect_marker(_image_msg, *cloud_msg);
    else
	_cloud_msg = *cloud_msg;
}

void
ArucoSimple::detect_marker(const image_t& image_msg, const cloud_t& cloud_msg)
{
    if ((_image_pub.getNumSubscribers()	    == 0) &&
	(_debug_pub.getNumSubscribers()	    == 0) &&
	(_pose_pub.getNumSubscribers()	    == 0) &&
	(_transform_pub.getNumSubscribers() == 0) &&
	(_position_pub.getNumSubscribers()  == 0) &&
	(_marker_pub.getNumSubscribers()    == 0) &&
	(_pixel_pub.getNumSubscribers()	    == 0))
    {
	ROS_DEBUG("No subscribers, not looking for aruco markers");
	return;
    }

    if (!_camParam.isValid())
	return;

    try
    {
      //const auto	curr_stamp = ros::Time::now();
	const auto	curr_stamp = image_msg.header.stamp;
	auto		inImage = cv_bridge::toCvCopy(
					image_msg,
					sensor_msgs::image_encodings::RGB8)
				->image;

      //detection results will go into "markers"
	std::vector<aruco::Marker>	markers;
	_mDetector.detect(inImage, markers, _camParam, _marker_size, false);
	    
      //for each marker, draw info and its boundaries in the image
	for (const auto& marker : markers)
	{
	  // only publishing the selected marker
	    if (marker.id == _marker_id)
	    {
		get_transform(marker, cloud_msg);
		
		tf::Transform		transform = arucoMarker2Tf(marker);

		tf::StampedTransform	cameraToReference;
		cameraToReference.setIdentity();
		if (_reference_frame != _camera_frame)
		    get_transform(_reference_frame, _camera_frame,
				  cameraToReference);

		transform = static_cast<tf::Transform>(cameraToReference) 
		    * static_cast<tf::Transform>(_rightToLeft) 
		    * transform;

		tf::StampedTransform
		    stampedTransform(transform, curr_stamp,
				     _reference_frame, _marker_frame);
		_tfBroadcaster.sendTransform(stampedTransform);

		geometry_msgs::PoseStamped	poseMsg;
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

		geometry_msgs::PointStamped	pixelMsg;
		pixelMsg.header  = transformMsg.header;
		pixelMsg.point.x = marker.getCenter().x;
		pixelMsg.point.y = marker.getCenter().y;
		pixelMsg.point.z = 0;
		_pixel_pub.publish(pixelMsg);

	      //Publish rviz marker representing the ArUco marker patch
		visualization_msgs::Marker	visMarker;
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
		aruco::CvDrawingUtils::draw3dAxis(inImage, marker, _camParam);
	}

	if (_image_pub.getNumSubscribers() > 0)
	{
	  //show input with augmented information
	    cv_bridge::CvImage	out_msg;
	    out_msg.header.stamp = curr_stamp;
	    out_msg.encoding     = sensor_msgs::image_encodings::RGB8;
	    out_msg.image	 = inImage;
	    _image_pub.publish(out_msg.toImageMsg());
	}

	if (_debug_pub.getNumSubscribers() > 0)
	{
	  //show also the internal image
	  //resulting from the threshold operation
	    cv_bridge::CvImage	debug_msg;
	    debug_msg.header.stamp = curr_stamp;
	    debug_msg.encoding     = sensor_msgs::image_encodings::MONO8;
	    debug_msg.image	   = _mDetector.getThresholdedImage();
	    _debug_pub.publish(debug_msg.toImageMsg());
	}
    }
    catch (const cv_bridge::Exception& e)
    {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
    }
}

bool
ArucoSimple::get_transform(const std::string& refFrame,
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

void
ArucoSimple::get_transform(const aruco::Marker& marker,
			   const cloud_t& cloud_msg) const
{
    using T		= float;
    using point_t	= cv::Vec<T, 3>;
    using iterator_t	= sensor_msgs::PointCloud2ConstIterator<T>;
    using rgbiterator_t	= sensor_msgs::PointCloud2Iterator<uint32_t>;


  // Compute initial marker plane.
    std::vector<point_t>	points;
    for (const auto& corner : marker)
    {
	const auto	point = at<T>(cloud_msg, corner.x, corner.y);
	if (!std::isnan(point(2)))
	{
	    std::cerr << "at(" << corner.x << ", " << corner.y << "): " << point
		      << std::endl;
	    points.push_back(point);
	}
    }

    if (points.size() < 3)
    {
	std::cerr << "Failed to find initial plane." << std::endl;
	return;
    }

    std::cerr << "=== Initial plane ===" << std::endl;
    auto	plane = fit_plane(points.cbegin(), points.cend());

  // Compute 2D bounding box of marker.
    const int	u0 = std::floor(std::min({marker[0].x, marker[1].x,
					  marker[2].x, marker[3].x}));
    const int	v0 = std::floor(std::min({marker[0].y, marker[1].y,
					  marker[2].y, marker[3].y}));
    const int	u1 = std::ceil( std::max({marker[0].x, marker[1].x,
					  marker[2].x, marker[3].x}));
    const int	v1 = std::ceil( std::max({marker[0].y, marker[1].y,
					  marker[2].y, marker[3].y}));

  // Select 3D points near to the initial plane within the bounding box.
    points.clear();
    for (auto v = v0; v <= v1; ++v)
    {
	iterator_t	xyz(cloud_msg, "x");
	rgbiterator_t	rgb(const_cast<cloud_t&>(cloud_msg), "rgb");
	const auto	idx = cloud_msg.width * v + u0;
	xyz += idx;
	rgb += idx;
	
	for (auto u = u0; u <= u1; ++u)
	{
	    const auto	point = at<T>(cloud_msg, u, v);
	    
	    if (!std::isnan(point(2)) && distance(plane, point) < 0.001)
	    {
		points.push_back(point);
		*rgb = 0xff0000;
	    }

	    ++xyz;
	    ++rgb;
	}
    }

    if (points.size() < 3)
    {
	std::cerr << "Failed to find refined plane." << std::endl;
	return;
    }

    std::cerr << "=== Refined plane ===" << std::endl;
    plane = fit_plane(points.cbegin(), points.cend());
    
    _cloud_pub.publish(cloud_msg);
}
    
}	// namespace aruco_ros

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_simple");

    aruco_ros::ArucoSimple	node;
    ros::spin();

    return 0;
}
