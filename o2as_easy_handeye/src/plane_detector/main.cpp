/*!
 *  \file main.cpp
 */
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <dynamic_reconfigure/server.h>

#include <o2as_easy_handeye/o2as_easy_handeyeConfig.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

namespace o2as_easy_handeye
{
/************************************************************************
*  class PlaneDetector							*
************************************************************************/
class PlaneDetector
{
  private:
    using	cloud_t = sensor_msgs::PointCloud2;
    using	cloud_p	= sensor_msgs::PointCloud2ConstPtr;
    
  public:
    PlaneDetector();

  private:
    void	cloud_callback(const cloud_p& cloud_msg)		;
    void	reconf_callback(o2as_easy_handeyeConfig& config,
				uint32_t level)				;
    
  private:
    ros::NodeHandle		_nh;

    const ros::Subscriber	_cloud_sub;
    const ros::Publisher	_cloud_pub;
    const ros::Publisher	_transform_pub; 

    dynamic_reconfigure::Server<o2as_easy_handeyeConfig>
				_reconf_server;

    float			_planarity_tolerance;
};
    
PlaneDetector::PlaneDetector()
    :_nh("~"),
     _cloud_sub(_nh.subscribe("/pointcloud", 1,
			      &PlaneDetector::cloud_callback, this)),
     _cloud_pub(_nh.advertise<cloud_t>("pointcloud", 1)),
     _transform_pub(_nh.advertise<geometry_msgs::TransformStamped>(
			"transform", 100)),
     _reconf_server(_nh),
     _planarity_tolerance(0.008)
{
    _nh.param<float>("tolerance", _planarity_tolerance, 0.008);
    _reconf_server.setCallback(boost::bind(&PlaneDetector::reconf_callback,
					   this, _1, _2));
}

void
PlaneDetector::cloud_callback(const cloud_p& cloud_msg)
{
    using pcl_point_t	= pcl::PointXYZRGB;
    using pcl_cloud_t	= pcl::PointCloud<pcl_point_t>;
    
    pcl_cloud_t::Ptr		cloud(new pcl_cloud_t);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    pcl::SACSegmentation<pcl_point_t>	seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(_planarity_tolerance);

    pcl::ModelCoefficients	coefficients;
    pcl::PointIndices		inliers;
    seg.setInputCloud(cloud);
    seg.segment(inliers, coefficients);

    auto&	plane = coefficients.values;
    if (plane[3] < 0)
    {
	plane[0] *= -1;
	plane[1] *= -1;
	plane[2] *= -1;
	plane[3] *= -1;
    }
    std::cerr << "Plane: ("
	      << plane[0] << ' ' << plane[1] << ' '
	      << plane[2] << ' ' << plane[3] << ')' << std::endl;

    for (size_t i = 0; i < inliers.indices.size(); ++i)
    {
	const uint32_t	rgb = 0xff0000;
	auto&		point = cloud->points[inliers.indices[i]];
	point.rgb = *reinterpret_cast<const float*>(&rgb);
    }

    cloud_t	out_msg;
    pcl::toROSMsg(*cloud, out_msg);
    out_msg.header.stamp = ros::Time::now();
    out_msg.header.frame_id = "map";
    _cloud_pub.publish(out_msg);
}

void
PlaneDetector::reconf_callback(o2as_easy_handeyeConfig& config, uint32_t level)
{
    if (level & (1 << 0))
	_planarity_tolerance = config.planarity_tolerance;
}
    
}	// namespace o2as_easy_handeye

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "o2as_plane_detector");

    try
    {
	o2as_easy_handeye::PlaneDetector	node;
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
