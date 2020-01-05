/*!
 *  \file	DepthFilter.cpp
 *  \author	Toshio UESHIBA
 *  \brief	Thin wraper of Photoneo Localization SDK
 */
#include <tiffio.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/iterator/iterator_adaptor.hpp>
#include "DepthFilter.h"
#include "oply/OrderedPly.h"

namespace aist_depth_filter
{
/************************************************************************
*  struct RGB								*
************************************************************************/
struct RGB	{ uint8_t r, g, b; };

/************************************************************************
*  global functions to save/load TIFF images				*
************************************************************************/
template <class T>
float	fval(const T* p)	{ return *p; }
float	fval(const uint16_t* p)	{ return 0.001f * *p; }
float	fval(const RGB* p)	{ return 0.3f*p->r + 0.59*p->g + 0.11*p->b; }
    
template <class T> T*
ptr(sensor_msgs::Image& image, int v)
{
    return reinterpret_cast<T*>(image.data.data() + v*image.step);
}

template <class T> const T*
ptr(const sensor_msgs::Image& image, int v)
{
    return reinterpret_cast<const T*>(image.data.data() + v*image.step);
}

/************************************************************************
*  global functions to save/load TIFF images				*
************************************************************************/
void
saveTiff(const sensor_msgs::Image& image, const std::string& file)
{
    using	namespace sensor_msgs;

    TIFFDataType	dataType;
    int			bitsPerSample;
    int			samplesPerPixel;

    if (image.encoding == image_encodings::TYPE_16UC1)
    {
	dataType	= TIFF_SHORT;
	bitsPerSample	= 16;
	samplesPerPixel	= 1;
    }
    else if (image.encoding == image_encodings::TYPE_32FC1)
    {
	dataType	= TIFF_FLOAT;
	bitsPerSample	= 32;
	samplesPerPixel = 1;
    }
    else
    {
	throw std::runtime_error("saveTiff(): unknown encoding["
				 + image.encoding + ']');
    }


    TIFF* const	tiff = TIFFOpen(file.c_str(), "w");
    if (!tiff)
	throw std::runtime_error("saveTiff(): cannot open file["
				 + file + ']');

    TIFFSetField(tiff, TIFFTAG_IMAGEWIDTH,	image.width);
    TIFFSetField(tiff, TIFFTAG_IMAGELENGTH,	image.height);
    TIFFSetField(tiff, TIFFTAG_BITSPERSAMPLE,	bitsPerSample);
    TIFFSetField(tiff, TIFFTAG_SAMPLESPERPIXEL,	samplesPerPixel);
    TIFFSetField(tiff, TIFFTAG_ROWSPERSTRIP,	1);
    TIFFSetField(tiff, TIFFTAG_COMPRESSION,	COMPRESSION_NONE);
    TIFFSetField(tiff, TIFFTAG_PHOTOMETRIC,	PHOTOMETRIC_MINISBLACK);
    TIFFSetField(tiff, TIFFTAG_FILLORDER,	FILLORDER_MSB2LSB);
    TIFFSetField(tiff, TIFFTAG_PLANARCONFIG,	PLANARCONFIG_CONTIG);
    TIFFSetField(tiff, TIFFTAG_ORIENTATION,	ORIENTATION_TOPLEFT);
    TIFFSetField(tiff, TIFFTAG_XRESOLUTION,	72.0);
    TIFFSetField(tiff, TIFFTAG_YRESOLUTION,	72.0);
    TIFFSetField(tiff, TIFFTAG_RESOLUTIONUNIT,	RESUNIT_CENTIMETER);

    for (int n = 0, offset = 0; n < image.height; ++n)
    {
	TIFFWriteEncodedStrip(tiff, n, image.data.data() + offset, image.step);
	offset += image.step;
    }

    TIFFClose(tiff);
}

sensor_msgs::Image
loadTiff(const std::string& file)
{
    using	namespace sensor_msgs;

    TIFF* const	tiff = TIFFOpen(file.c_str(), "r");
    if (!tiff)
	throw std::runtime_error("loadTiff(): cannot open file[" + file + ']');

    uint32	width, height;
    uint16	bitsPerSample, samplesPerPixel, photometric;

    if (!TIFFGetField(tiff, TIFFTAG_IMAGEWIDTH,	     &width)		||
	!TIFFGetField(tiff, TIFFTAG_IMAGELENGTH,     &height)		||
	!TIFFGetField(tiff, TIFFTAG_BITSPERSAMPLE,   &bitsPerSample)	||
	!TIFFGetField(tiff, TIFFTAG_SAMPLESPERPIXEL, &samplesPerPixel)	||
	!TIFFGetField(tiff, TIFFTAG_PHOTOMETRIC,     &photometric))
    {
	throw std::runtime_error("loadTiff(): cannot get necessary fields");
    }

    sensor_msgs::Image	image;
    image.width  = width;
    image.height = height;

    switch (photometric)
    {
      case PHOTOMETRIC_MINISBLACK:
	switch(bitsPerSample)
	{
	  case 8:
	    image.encoding = image_encodings::TYPE_8UC1;
	    break;
	  case 16:
	    image.encoding = image_encodings::TYPE_16UC1;
	    break;
	  case 32:
	    image.encoding = image_encodings::TYPE_32FC1;
	    break;
	  case 64:
	    image.encoding = image_encodings::TYPE_64FC1;
	    break;
	  default:
	    throw std::runtime_error("loadTiff(): unsupported #bits per sample["
				     + std::to_string(bitsPerSample) + ']');
	}
	break;

      case PHOTOMETRIC_RGB:
	switch(bitsPerSample)
	{
	  case 8:
	    image.encoding = image_encodings::TYPE_8UC3;
	    break;
	  case 16:
	    image.encoding = image_encodings::TYPE_16UC3;
	    break;
	  case 32:
	    image.encoding = image_encodings::TYPE_32FC3;
	    break;
	  case 64:
	    image.encoding = image_encodings::TYPE_64FC3;
	    break;
	  default:
	    throw std::runtime_error("loadTiff(): unsupported #bits per sample["
				     + std::to_string(bitsPerSample) + ']');
	}
	break;

      default:
	throw std::runtime_error("loadTiff(): unsupported photometic["
				 + std::to_string(photometric) + ']');
    }

    image.step = image.width * bitsPerSample * samplesPerPixel / 8;
    image.data.resize(image.height * image.step);

    const auto	nBytesPerStrip = TIFFStripSize(tiff);
    const auto	nStrips        = TIFFNumberOfStrips(tiff);

    for (int n = 0, offset = 0; n < nStrips; ++n)
    {
	const auto nBytes = TIFFReadEncodedStrip(tiff, n,
						 image.data.data() + offset,
						 nBytesPerStrip);
	if (nBytes < 0)
	    throw std::runtime_error("loadTiff(): failed to read strip");

	offset += nBytes;
    }

    TIFFClose(tiff);

    return image;
}

/************************************************************************
*  global functions to save Ordered PLY point cloud			*
************************************************************************/
namespace detail
{
  template <class ITER>
  class input_proxy
  {
    public:
      using value_type = typename std::iterator_traits<ITER>::value_type;
      
    public:
      input_proxy(ITER iter) :_iter(iter)	{}

      const auto&	operator =(const value_type& val) const
			{
			    *_iter = val;
			    return *this;
			}
      const auto&	operator =(const RGB& color) const
			{
			    _iter[0] = color.r;
			    _iter[1] = color.g;
			    _iter[2] = color.b;
			    return *this;
			}
      const auto&	operator =(uint16_t val) const
			{
			    *_iter = 0.001 * val;
			    return *this;
			}
	
    private:
      ITER 	_iter;
  };
}	// namespace detail
    
template <class ITER>
class input_iterator
    : public boost::iterator_adaptor<input_iterator<ITER>,
				     ITER,
				     boost::use_default,
				     boost::use_default,
				     detail::input_proxy<ITER> >
{
  private:
    using super = boost::iterator_adaptor<input_iterator<ITER>,
					  ITER,
					  boost::use_default,
					  boost::use_default,
					  detail::input_proxy<ITER> >;
    friend	class boost::iterator_core_access;

  public:
    using	typename super::reference;
    
  public:
    input_iterator(ITER iter) :super(iter)	{}

  private:
    reference	dereference()		const	{ return {super::base()}; }
};

template <class ITER> input_iterator<ITER>
make_input_iterator(ITER iter)	{ return input_iterator<ITER>(iter); }
    
template <class T, class ITER> ITER
copy_image(const sensor_msgs::Image& image, ITER iter)
{
    using value_type = typename std::iterator_traits<ITER>::value_type;
    
    for (int v = 0; v < image.height; ++v)
    {
	auto	p = ptr<T>(image, v);
	for (auto q = p + image.width; p != q; ++p, ++iter)
	    *iter = *p;
    }
    
    return iter;
}
    
template <class T>
using array3_t = std::array<T, 3>;
    
void
saveAsOPly(const sensor_msgs::CameraInfo& camera_info,
	   const sensor_msgs::Image& image, const sensor_msgs::Image& depth,
	   const std::string& file)
{
    using	namespace sensor_msgs;
    
    OrderedPly	oply;

  // Set number of points.
    oply.size = depth.height * depth.width;
	
  // Set point 3D coordinates.
    oply.point.resize(oply.size);

  // Set normals.
    oply.normal.resize(oply.size);
    std::fill(make_input_iterator(oply.normal.begin()),
	      make_input_iterator(oply.normal.end()),
	      array3_t<float>({0.0f, 0.0f, 1.0f}));
    
  // Set colpr.
    oply.color.resize(oply.size);
    if (image.encoding == image_encodings::TYPE_32FC1)
	copy_image<float>(image, make_input_iterator(oply.color.begin()));
    else if (image.encoding == image_encodings::TYPE_8UC3)
	copy_image<RGB>(image, make_input_iterator(oply.color.begin()));
    
  // Set texture values.
    oply.texture.resize(oply.size);
    if (image.encoding == image_encodings::TYPE_32FC1)
	copy_image<float>(image, oply.texture.begin());
    else if (image.encoding == image_encodings::TYPE_8UC3)
	copy_image<RGB>(image, oply.texture.begin());
    else
	throw std::runtime_error("saveAsOPly(): unknown image encoding["
				 + image.encoding + ']');

  // Set depth values.
    oply.depth.resize(oply.size);
    if (depth.encoding == image_encodings::TYPE_16UC1)
	copy_image<uint16_t>(depth, oply.depth.begin());
    else if (depth.encoding == image_encodings::TYPE_32FC1)
	copy_image<float>(depth, oply.depth.begin());
    else
	throw std::runtime_error("saveAsOPly(): unknown depth encoding["
				 + depth.encoding + ']');

  // Set confidence values.
    oply.confidence.resize(oply.size);
    std::fill(oply.confidence.begin(), oply.confidence.end(), 0.0f);
    
  // Set camera orientation.
    oply.view   = {0.0, 0.0, 1.0};
    oply.x_axis = {1.0, 0.0, 0.0};
    oply.y_axis = {0.0, 1.0, 0.0};
    oply.z_axis = {0.0, 0.0, 1.0};

  // Set frame size.
    oply.frame_width  = depth.width;
    oply.frame_height = depth.height;

  // Copy camera parameters.
    std::copy(std::begin(camera_info.K), std::end(camera_info.K),
	      std::begin(oply.cm));
    std::copy(std::begin(camera_info.D), std::end(camera_info.D),
	      std::begin(oply.dm));

  // Other parameters.
    oply.width  = depth.width;
    oply.height = depth.height;

  // Write Ordered PLY to the spceified file.
    OPlyWriter	writer(file, oply);
    writer.write();
}
    
/************************************************************************
*  class DepthFilter							*
************************************************************************/
DepthFilter::DepthFilter(const std::string& name)
    :_nh(name),
     _saveBG_srv(_nh.advertiseService("saveBG", &saveBG_cb, this)),
     _saveAsOPly_srv(_nh.advertiseService("saveAsOPly", &saveAsOPly_cb, this)),
     _camera_info_sub(_nh, "/camera_info", 1),
     _image_sub( _nh, "/image", 1),
     _depth_sub( _nh, "/depth", 1),
     _sync(sync_policy_t(10), _camera_info_sub, _image_sub, _depth_sub),
     _it(_nh),
     _image_pub(_it.advertise("image", 1)),
     _depth_pub(_it.advertise("depth", 1)),
     _camera_info_pub(_nh.advertise<camera_info_t>("camera_info", 1)),
     _ddr(),
     _camera_info(nullptr),
     _image(nullptr),
     _depth(nullptr),
     _filtered_depth(),
     _bg_depth(),
     _threshBG(0.0),
     _fileBG("bg.tif"),
     _depth_clip(false),
     _near(0.0),
     _far(std::numeric_limits<double>::max()),
     _roi(false),
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
				  0.0, 1.0);
    _nh.param("depth_clip", _depth_clip, false);
    _ddr.registerVariable<bool>("depth_clip", &_depth_clip,
				"Clip outside of [near, far]");
    _nh.param("near", _near, 0.0);
    _ddr.registerVariable<double>("near", &_near,
				  "Nearest depth value", 0.0, 1.0);
    _nh.param("far", _far, 100.0);
    _ddr.registerVariable<double>("far", &_far,
				  "Farest depth value", 0.05, 4.0);
    _nh.param("roi", _roi, false);
    _ddr.registerVariable<bool>("roi", &_roi, "Mask outside of ROI");
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
DepthFilter::saveAsOPly_cb(std_srvs::Trigger::Request&  req,
			   std_srvs::Trigger::Response& res)
{
    try
    {
	if (!_filtered_depth)
	    throw std::runtime_error("no filtered depth image available!");

	saveAsOPly(*_camera_info, *_image, *_filtered_depth, _fileOPly);
	_filtered_depth = nullptr;

	res.success = true;
	res.message = "succeeded.";
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("DepthFilter::saveAsOPly_cb(): " << err.what());

	res.success = false;
	res.message = "failed.";
    }

    ROS_INFO_STREAM("(DepthFilter) save as OrderedPly: " << res.message);

    return true;
}

void
DepthFilter::filter_cb(const camera_info_cp& camera_info,
		       const image_cp& image, const image_cp& depth)
{
    _top    = std::max(0,     std::min(_top,    int(image->height)));
    _bottom = std::max(_top,  std::min(_bottom, int(image->height)));
    _left   = std::max(0,     std::min(_left,   int(image->width)));
    _right  = std::max(_left, std::min(_right,  int(image->width)));

    try
    {
	using	namespace sensor_msgs;

	_camera_info	= camera_info;
	_image		= image;
	_depth		= depth;
	_filtered_depth.reset(new image_t(*depth));

	if (depth->encoding == image_encodings::TYPE_16UC1)
	    filter<uint16_t>(*camera_info, *image, *_filtered_depth);
	else if (depth->encoding == image_encodings::TYPE_32FC1)
	    filter<float>(*camera_info, *image, *_filtered_depth);
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM("DepthFilter::filter_cb(): " << err.what());
    }

    _camera_info_pub.publish(*camera_info);
    _image_pub.publish(*image);
    _depth_pub.publish(*_filtered_depth);
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
		_bg_depth.reset(new image_t(loadTiff(_fileBG.c_str())));

	    removeBG<T>(depth, *_bg_depth);
	}
	catch (const std::exception& err)
	{
	    _bg_depth = nullptr;
	    _threshBG = 0;
	}
    }
    if (_depth_clip)
	depth_clip<T>(depth);
    if (_roi)
	roi<T>(depth);
    if (_scale != 1.0)
	scale<T>(depth);
}

template <class T> void
DepthFilter::removeBG(image_t& depth, const image_t& bg_depth) const
{
    if ((depth.width	!= bg_depth.width)	 ||
	(depth.height	!= bg_depth.height)	 ||
	(depth.encoding != bg_depth.encoding))
	throw std::runtime_error("DepthFilter::removeBG(): mismatched image properties!");

    for (int v = 0; v < depth.height; ++v)
    {
	auto p = ptr<T>(depth, v);
	auto b = ptr<T>(bg_depth, v);
	for (auto q = p + depth.width; p != q; ++p, ++b)
	    if (std::abs(fval(p) - fval(b)) < _threshBG)
		*p = 0;
    }
}

template <class T> void
DepthFilter::depth_clip(image_t& depth) const
{
    for (int v = 0; v < depth.height; ++v)
    {
	auto p = ptr<T>(depth, v);
	for (auto q = p + depth.width; p != q; ++p)
	    if (fval(p) < _near || fval(p) > _far)
		*p = 0;
    }
}

template <class T> void
DepthFilter::roi(image_t& depth) const
{
    for (int v = 0; v < _top; ++v)
    {
	auto p = ptr<T>(depth, v);
	for (auto q = p + depth.width; p != q; ++p)
	    *p = 0;
    }
    for (int v = _top; v < _bottom; ++v)
    {
	auto p = ptr<T>(depth, v);
	for (auto q = p + _left; p != q; ++p)
	    *p = 0;

	p = ptr<T>(depth, v) + _right;
	for (auto q = p - _right + depth.width; p != q; ++p)
	    *p = 0;
    }
    for (int v = _bottom; v < depth.height; ++v)
    {
	auto p = ptr<T>(depth, v);
	for (auto q = p + depth.width; p != q; ++p)
	    *p = 0;
    }
}

template <class T> void
DepthFilter::scale(image_t& depth) const
{
    for (int v = 0; v < depth.height; ++v)
    {
	auto p = ptr<T>(depth, v);
	for (auto q = p + depth.width; p != q; ++p)
	    *p *= _scale;
    }
}
    
}	// namespace aist_photoneo_localization
