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
*  geometry functions							*
************************************************************************/
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

template <class T, int N> T
angle(const cv::Vec<T, N>& x, const cv::Vec<T, N>& y)
{
    return std::acos(x.dot(y)/(cv::norm(x)*cv::norm(y))) * 180.0/M_PI;
}

template <class T> cv::Matx<T, 3, 3>
rodrigues(const cv::Matx<T, 3, 1>& r)
{
    using matrix_t = cv::Matx<T, 3, 3>;
    
    const auto	theta = r.norm();
    r /= theta;
    const auto	c = std::cos(theta), s = std::sin(theta);
    matrix_t	R = matrix_t::zeros();
    for (int i = 0; i < 3; ++i)
    {
	R(i, i) += c;
	
	for (int j = 0; j < 3; ++j)
	    R(i, j) += (1 - c)*r(i)*r(j);
    }
    R(0, 1) -= s * r(2);
    R(0, 2) += s * r(1);
    R(1, 0) += s * r(2);
    R(1, 2) -= s * r(0);
    R(2, 0) -= s * r(1);
    R(2, 1) += s * r(0);

    return R;
}
    
/************************************************************************
*  struct Plane<T, N>							*
************************************************************************/
template <class T, size_t N>
class Plane
{
  public:
    using vector_type	= cv::Vec<T, N>;
    using value_type	= T;

  public:
    Plane()					    :_n(), _d(0)	{}
    Plane(const vector_type& norm, value_type dist) :_n(norm), _d(dist)	{}
    template <class ITER>
    Plane(ITER begin, ITER end)				{ fit(begin, end); }
    
    template <class ITER>
    void		fit(ITER begin, ITER end)	;
    
    const vector_type&	normal()			const	{ return _n; }
    value_type		distance()			const	{ return _d; }
    value_type		distance(const vector_type& point) const
			{
			    return std::abs(_n.dot(point) + _d);
			}
    vector_type		cross_point(const vector_type& view_vector) const
			{
			    return (-_d/_n.dot(view_vector)) * view_vector;
			}

    friend std::ostream&
			operator <<(std::ostream& out, const Plane& plane)
			{
			    out << plane._n << ": " << plane._d;
			}

  private:
    vector_type	_n;	// normal
    value_type	_d;	// distance from the origin
};
    
template <class T, size_t N> template <class ITER> void
Plane<T, N>::fit(ITER begin, ITER end)
{
    using matrix_type	= cv::Matx<T, N, N>;

    const auto	ext   =	[](const auto& x)
			{
			    matrix_type	y;
			    for (size_t i = 0; i < N; ++i)
				for (size_t j = 0; j < N; ++j)
				    y(i, j) = x(i) * x(j);
			    return y;
			};

  // Check #points.
    const auto	npoints  = std::distance(begin, end);
    if (npoints < 3)
	throw std::runtime_error("Plane<T, N>::fit(): three or more points required!");
    
  // Compute centroid.
    auto	centroid = vector_type::zeros();
    for (auto iter = begin; iter != end; ++iter)
	centroid += *iter;
    centroid *= value_type(1)/value_type(npoints);

  // Compute moment matrix.
    auto	moments = matrix_type::zeros();
    for (auto iter = begin; iter != end; ++iter)
	moments += ext(*iter - centroid);

    matrix_type	evectors;
    vector_type	evalues;
    cv::eigen(moments, evalues, evectors);

    _n = vector_type::all(0);
    for (size_t j = 0; j < N; ++j)
	_n(j) = evectors(N - 1, j);
    _d = -_n.dot(centroid);
    if (_d < 0)
    {
	_n *= value_type(-1);
	_d *= -1;
    }

    ROS_DEBUG_STREAM("plane = " << *this << ", err = "
		     << std::sqrt(std::abs(evalues(N-1))/value_type(npoints))
		     << ", computed from " << npoints << " points.");
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
    tf::Transform
		get_marker_transform(const aruco::Marker& marker,
				     const cloud_t& cloud_msg)	const	;
    template <class T> cv::Vec<T, 3>
		view_vector(T u, T v)				const	;
    
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
     _cloud_pub(_nh.advertise<cloud_t>("pointcloud", 1)),
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

    ROS_INFO_STREAM("Aruco node started with marker size of "
		    << _marker_size << " m and marker id to track: "
		    << _marker_id);
    ROS_INFO_STREAM("Aruco node will publish pose to TF with "
		    << _reference_frame << " as parent and "
		    << _marker_frame << " as child.");

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
	ROS_WARN_STREAM("normalizeImageIllumination is unimplemented!");
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
	ROS_DEBUG_STREAM("No subscribers, not looking for aruco markers");
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
		tf::Transform	transform = get_marker_transform(marker,
								 cloud_msg);
	      //tf::Transform	transform = arucoMarker2Tf(marker);

		tf::StampedTransform	cameraToReference;
		cameraToReference.setIdentity();
		if (_reference_frame != _camera_frame)
		    get_transform(_reference_frame, _camera_frame,
				  cameraToReference);

		transform = static_cast<tf::Transform>(cameraToReference) 
			  * static_cast<tf::Transform>(_rightToLeft) 
			  * transform;

		const tf::StampedTransform
		    stampedTransform(transform, curr_stamp,
				     _reference_frame, _marker_frame);
		_tfBroadcaster.sendTransform(stampedTransform);

		geometry_msgs::PoseStamped	poseMsg;
		tf::poseTFToMsg(transform, poseMsg.pose);
		poseMsg.header.frame_id = _reference_frame;
		poseMsg.header.stamp    = curr_stamp;
		_pose_pub.publish(poseMsg);

		geometry_msgs::TransformStamped	transformMsg;
		tf::transformStampedTFToMsg(stampedTransform, transformMsg);
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
	ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
	return;
    }
    catch (const std::runtime_error& e)
    {
	ROS_WARN_STREAM("Failed to compute marker plane: " << e.what());
    }
    catch (...)
    {
	ROS_ERROR_STREAM("Unknown error");
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

tf::Transform
ArucoSimple::get_marker_transform(const aruco::Marker& marker,
				  const cloud_t& cloud_msg) const
{
    using value_t	= float;
    using iterator_t	= sensor_msgs::PointCloud2ConstIterator<value_t>;
    using rgbiterator_t	= sensor_msgs::PointCloud2Iterator<uint32_t>;
    using point_t	= cv::Vec<value_t, 3>;
    using plane_t	= Plane<value_t, 3>;

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
	    const auto	idx = cloud_msg.width * v + u0;
	    xyz += idx;
	    rgb += idx;
	
	    for (auto u = u0; u <= u1; ++u)
	    {
		const auto	point = at<value_t>(cloud_msg, u, v);
	    
		if (!std::isnan(point(2)) && plane.distance(point) < 0.001)
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
	    
		if (!std::isnan(point(2)) && plane.distance(point) < 0.001)
		    points.push_back(point);
		
		++xyz;
	    }
	}
    }

    plane.fit(points.cbegin(), points.cend());

    _cloud_pub.publish(cloud_msg);

  // Compute 3D coordinates of marker corners.
    point_t	corners[4];
    for (int i = 0; i < 4; ++i)
	corners[i] = plane.cross_point(view_vector(marker[i].x, marker[i].y));

  // Compute p and q, i.e. marker's local x-axis and y-axis respectively.
    const auto&	n = plane.normal();
    const auto	c = (corners[2] + corners[3] - corners[0] - corners[1])
		  + (corners[1] + corners[2] - corners[3] - corners[0])
			.cross(n);
    const auto	p = c / cv::norm(c);
    const auto	q = n.cross(p);

  // Compute marker centroid.
    const auto	centroid = 0.25*(corners[0] + corners[1] +
				 corners[2] + corners[3]);

    return tf::Transform(tf::Matrix3x3(-p(0), n(0), q(0),
				       -p(1), n(1), q(1),
				       -p(2), n(2), q(2)),
			 tf::Vector3(centroid(0), centroid(1), centroid(2)));
}

template <class T> cv::Vec<T, 3>
ArucoSimple::view_vector(T u, T v) const
{
    const auto&	K = _camParam.CameraMatrix;
    const auto	y = (v - K.at<T>(1, 2))/K.at<T>(1, 1);
    
    return {(u - K.at<T>(0, 2) - y*K.at<T>(0, 1))/K.at<T>(0, 0), y, T(1)};
}
    
}	// namespace aruco_ros

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_simple");

    try
    {
	aruco_ros::ArucoSimple	node;
	ros::spin();
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM(err.what());
	return 1;
    }
    catch (...)
    {
	ROS_ERROR_STREAM("Unknown error.");
    }
    
    return 0;
}
