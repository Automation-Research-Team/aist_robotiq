/*!
 *  \file	DepthFilter.cpp
 *  \author	Toshio UESHIBA
 *  \brief	Thin wraper of Photoneo Localization SDK
 */
#include <sensor_msgs/image_encodings.h>
#include "tiff.h"
#include "ply.h"
#include "utils.h"
#include "DepthFilter.h"

namespace aist_depth_filter
{
/************************************************************************
*  class DepthFilter							*
************************************************************************/
DepthFilter::DepthFilter(const std::string& name)
    :_nh(name),
     _saveBG_srv(_nh.advertiseService("saveBG", &saveBG_cb, this)),
     _savePly_srv(_nh.advertiseService("savePly", &savePly_cb, this)),
     _camera_info_sub(_nh, "/camera_info", 1),
     _image_sub( _nh, "/image",  1),
     _depth_sub( _nh, "/depth",  1),
     _normal_sub(_nh, "/normal", 1),
     _sync(sync_policy_t(10),
	   _camera_info_sub, _image_sub, _depth_sub, _normal_sub),
     // _sync2(sync_policy2_t(10),
     // 	    _camera_info_sub, _image_sub, _depth_sub),
     _it(_nh),
     _image_pub (_it.advertise("image",  1)),
     _depth_pub( _it.advertise("depth",  1)),
     _normal_pub(_it.advertise("normal", 1)),
     _camera_info_pub(_nh.advertise<camera_info_t>("camera_info", 1)),
     _ddr(),
     _camera_info(),
     _image(),
     _depth(nullptr),
     _bg_depth(nullptr),
     _filtered_depth(),
     _normal(),
     _threshBG(0.0),
     _fileBG("bg.tif"),
     _near(0.0),
     _far(FarMax),
     _top(0),
     _bottom(2048),
     _left(0),
     _right(3072),
     _scale(1.0),
     _fileOPly("filtered_depth.ply")
{
    _nh.param("thresh_bg", _threshBG, 0.0);
    _ddr.registerVariable<double>("thresh_bg", &_threshBG,
				  "Threshold value for background removal",
				  0.0, 0.1);
    _nh.param("near", _near, 0.0);
    _ddr.registerVariable<double>("near", &_near,
				  "Nearest depth value", 0.0, 1.0);
    _nh.param("far", _far, 100.0);
    _ddr.registerVariable<double>("far", &_far,
				  "Farest depth value", 0.0, FarMax);
    _nh.param("top", _top, 0);
    _ddr.registerVariable<int>("top",    &_top,	   "Top of ROI",    0, 2048);
    _nh.param("bottom", _bottom, 2048);
    _ddr.registerVariable<int>("bottom", &_bottom, "Bottom of ROI", 0, 2048);
    _nh.param("left", _left, 0);
    _ddr.registerVariable<int>("left",   &_left,   "Left of ROI",   0, 3072);
    _nh.param("right", _right, 3072);
    _ddr.registerVariable<int>("right",  &_right,  "Right of ROI",  0, 3072);
    _nh.param("scale", _scale, 1.0);
    _ddr.registerVariable<double>("scale", &_scale, "Scale depth", 0.5, 1.5);
    _ddr.publishServicesTopics();

    _sync.registerCallback(&DepthFilter::filter_cb, this);
}

void
DepthFilter::run()
{
    ros::spin();
}

bool
DepthFilter::saveBG_cb(std_srvs::Trigger::Request&  req,
		       std_srvs::Trigger::Response& res)
{
    try
    {
	if (!_depth)
	    throw std::runtime_error("no original depth image available!");

	saveTiff(*_depth, _fileBG);
	_bg_depth = _depth;
	_depth	  = nullptr;

	res.success = true;
	res.message = "succeeded.";
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("DepthFilter::saveBG_cb(): " << err.what());

	res.success = false;
	res.message = "failed.";
    }

    ROS_INFO_STREAM("(DepthFilter) save background image to " + _fileBG + ": "
		    << res.message);

    return true;
}

bool
DepthFilter::savePly_cb(std_srvs::Trigger::Request&  req,
			std_srvs::Trigger::Response& res)
{
    try
    {
	if (_filtered_depth.data.empty())
	    throw std::runtime_error("no filtered depth image available!");

	savePly(_camera_info, _image, _filtered_depth, _normal, _fileOPly);
	_filtered_depth.data.clear();

	res.success = true;
	res.message = "succeeded.";
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("DepthFilter::savePly_cb(): " << err.what());

	res.success = false;
	res.message = "failed.";
    }

    ROS_INFO_STREAM("(DepthFilter) save as OrderedPly: " << res.message);

    return true;
}

void
DepthFilter::filter_cb(const camera_info_cp& camera_info, const image_cp& image,
		       const image_cp& depth, const image_cp& normal)
{
    _top    = std::max(0,     std::min(_top,    int(image->height)));
    _bottom = std::max(_top,  std::min(_bottom, int(image->height)));
    _left   = std::max(0,     std::min(_left,   int(image->width)));
    _right  = std::max(_left, std::min(_right,  int(image->width)));

    try
    {
	using	namespace sensor_msgs;

      // Create camera_info according to ROI.
	_camera_info	    = *camera_info;
	_camera_info.height = _bottom - _top;
	_camera_info.width  = _right  - _left;
	_camera_info.K[2]  -= _left;
	_camera_info.K[5]  -= _top;
	_camera_info.P[2]  -= _left;
	_camera_info.P[6]  -= _top;
	
	_depth = depth;  // Keep pointer to depth for saving background.
	create_subimage(*image,  _image);
	create_subimage(*depth,  _filtered_depth);
	create_subimage(*normal, _normal);
	
	if (depth->encoding == image_encodings::MONO16 ||
	    depth->encoding == image_encodings::TYPE_16UC1)
	    filter<uint16_t>(_camera_info, _image, _filtered_depth);
	else if (depth->encoding == image_encodings::TYPE_32FC1)
	    filter<float>(_camera_info, _image, _filtered_depth);

	_camera_info_pub.publish(_camera_info);
	_image_pub.publish(_image);
	_depth_pub.publish(_filtered_depth);
	_normal_pub.publish(_normal);
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("DepthFilter::filter_cb(): " << err.what());
    }
}

template <class T> void
DepthFilter::filter(const camera_info_t& camera_info,
		    const image_t& image, image_t& depth)
{
    if (_threshBG > 0)
    {
	try
	{
	    if (!_bg_depth)
		_bg_depth = loadTiff(_fileBG.c_str());

	    removeBG<T>(depth, *_bg_depth);
	}
	catch (const std::exception& err)
	{
	    _bg_depth = nullptr;
	    _threshBG = 0;
	}
    }
    if (_near > 0.0 || _far < FarMax)
    {
	z_clip<T>(depth);
    }
    if (_scale != 1.0)
    {
	scale<T>(depth);
    }
}

template <class T> void
DepthFilter::removeBG(image_t& depth, const image_t& bg_depth) const
{
    for (int v = 0; v < depth.height; ++v)
    {
	auto	p = ptr<T>(depth, v);
	auto	b = ptr<T>(bg_depth, v + _top) + _left;
	for (const auto q = p + depth.width; p != q; ++p, ++b)
	    if (*b != 0 && std::abs(fval(*p) - fval(*b)) < _threshBG)
		*p = 0;
    }
}

template <class T> void
DepthFilter::z_clip(image_t& depth) const
{
    for (int v = 0; v < depth.height; ++v)
    {
	const auto	p = ptr<T>(depth, v);
	std::replace_if(p, p + depth.width,
			[this](const auto& val)
			{ return (fval(val) < _near || fval(val) > _far); },
			0);
    }
}
  /*
template <class T> DepthFilter::image_t
DepthFilter::computeNormal(const camera_info_t& camera_info,
			   const image_t& depth) const
{
    using		namespace sensor_msgs;
    using vector3_t	= cv::Vec<float, 3>;
    using matrix33_t	= cv::Matx<float, 3, 3>;
    
    image_t	normal;
    normal.encoding = image_encodings::TYPE_32FC3;
    normal.height   = depth.height;
    normal.width    = depth.width;
    normal.step	    = normal.width * sizeof(vector3_t);
    normal.data.resize(normal.height * normal.step);
    std::fill(normal.data.begin(), normal.data.end(), 0);

  // Compute 3D coordinates.
    std::vector<vector3_t>	xyz(depth.height * depth.width);
    depth_to_points<T>(camera_info, depth, xyz.begin());

  // Compute normals.
    std::vector<vector3_t>	win;
    for (int v = 0; v <= depth.height - _winSize; ++v)
    {
	auto	p = ptr<T>(depth, v);
	auto	r = ptr<vector3_t>(normal, v + _winRadius) + _winRadius;

	
	for (; ; )
	{
	}
    }
}
  */
template <class T> void
DepthFilter::scale(image_t& depth) const
{
    for (int v = 0; v < depth.height; ++v)
    {
	const auto	p = ptr<T>(depth, v);
	std::transform(p, p + depth.width, p,
		       [this](const auto& val){ return _scale * val; });
    }
}

void
DepthFilter::create_subimage(const image_t& image, image_t& subimage) const
{
    using	namespace sensor_msgs;

    constexpr auto	ALIGN = sizeof(float);
    const auto	nbytesPerPixel = image_encodings::bitDepth(image.encoding)/8
			       * image_encodings::numChannels(image.encoding);

    subimage.header	  = image.header;
    subimage.height	  = _bottom - _top;
    subimage.width	  = _right  - _left;
    subimage.encoding	  = image.encoding;
    subimage.is_bigendian = image.is_bigendian;
    subimage.step	  = subimage.width*nbytesPerPixel;
    subimage.data.resize(subimage.height * subimage.step);

    auto p = image.data.begin() + _top*image.step + _left*nbytesPerPixel;
    for (auto q = subimage.data.begin(); q != subimage.data.end();
	 q += subimage.step)
    {
	std::copy_n(p, subimage.width*nbytesPerPixel, q);
	p += image.step;
    }
}

}	// namespace aist_depth_filter
