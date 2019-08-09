/*!
 *  \file	ftsensor.cpp
 *  \brief	source file for a class for controlling FT300 force sensors
 */
#include "ftsensor.h"

namespace aist_ftsensor
{

/************************************************************************
*  class ftsensor								*
************************************************************************/
ftsensor::ftsensor(const std::string& name)
    :_nh(name),
     _publisher(_nh.advertise<wrench_t>("ftsensor_wrench", 100)),
     _subscriber(_nh.subscribe("/wrench", 100,
			       &ftsensor::wrench_callback, this)),
     _rate(100)
{
    _nh.param<std::string>("sensor_frame", _frame, "ftsensor_wrench_link");
    _nh.param<double>("rate", _rate, 100);
    ROS_INFO_STREAM("sensor_frame=" << _frame << ", rate=" << _rate);
    ROS_INFO_STREAM("aist_ftsensor started.");
}

ftsensor::~ftsensor()
{
    ;
}

void
ftsensor::run()
{
    ros::Rate	rate(_rate);

    while (ros::ok())
    {
	ros::spinOnce();
	rate.sleep();
    }
}

void
ftsensor::wrench_callback(const geometry_msgs::WrenchStampedConstPtr& msg)
{
    ROS_INFO("#Callback# sec - %d", msg->header.stamp.sec);

    try
    {
	wrench_p	wrench(new wrench_t);
	wrench->header = msg->header;
	wrench->wrench = msg->wrench;
	wrench->header.frame_id = _frame;

	_publisher.publish(wrench);
    }
    catch (const std::exception& err)
    {
	ROS_ERROR_STREAM(err.what());
    }
}

double
ftsensor::rate() const
{
    return _rate;
}

}	// namespace aist_ftsensor
