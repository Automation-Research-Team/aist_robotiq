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
     _publisher(_nh.advertise<wrench_t>("wrench", 100)),
     _subscriber(_nh.subscribe("/wrench_in", 100,
			       &ftsensor::wrench_callback, this)),
     _listener(),
     _reference_frame("workspace_center"),
     _sensor_frame("ftsensor_wrench_link"),
     _rate(100),
     _mg(0, 0, 0),
     _f_offset(0, 0, 0),
     _m_offset(0, 0, 0),
     _r_offset(0, 0, 0)
{
    _nh.param<std::string>("reference_frame", _reference_frame,
			   "workspace_center");
    _nh.param<std::string>("sensor_frame", _sensor_frame,
			   "ftsensor_wrench_link");
    _nh.param<double>("rate", _rate, 100);

    ROS_INFO_STREAM("reference_frame=" << _reference_frame <<
		    ", sensor_frame=" << _sensor_frame << ", rate=" << _rate);

    double	effector_mass;
    _nh.param<double>("effector_mass", effector_mass, 0);
    if (effector_mass > 0)
	_mg = tf::Vector3(0, 0, -9.8 * effector_mass);
    ROS_INFO_STREAM("mg=[" << _mg.x() << "," << _mg.y() << "," << _mg.z());

    std::vector<double> vec;
    if (_nh.getParam("f_offset", vec))
    {
	if (vec.size() >= 3)
	{
	    for (int i = 0; i < 3; i++)
		_f_offset.m_floats[i] = vec[i];
	}
    }
    ROS_INFO_STREAM("f_offset[" << _f_offset.x() << "," <<
		    _f_offset.y() << "," << _f_offset.z() << "]");

    vec.clear();
    if (_nh.getParam("m_offset", vec))
    {
	if (vec.size() >= 3)
	{
	    for (int i = 0; i < 3; i++)
		_m_offset.m_floats[i] = vec[i];
	}
    }
    ROS_INFO_STREAM("m_offset[" << _m_offset.x() << "," <<
		    _m_offset.y() << "," << _m_offset.z() << "]");

    vec.clear();
    if (_nh.getParam("r_offset", vec))
    {
	if (vec.size() >= 3)
	{
	    for (int i = 0; i < 3; i++)
		_r_offset.m_floats[i] = vec[i];
	}
    }
    ROS_INFO_STREAM("r_offset[" << _r_offset.x() << "," <<
		    _r_offset.y() << "," << _r_offset.z() << "]");

    ROS_INFO_STREAM("aist_ftsensor started.");
}

ftsensor::~ftsensor()
{
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
ftsensor::wrench_callback(const const_wrench_p& wrench_msg)
{
    // ROS_INFO("#Callback# sec - %d", wrench_msg->header.stamp.sec);

    try
    {
	_listener.waitForTransform(_sensor_frame, _reference_frame,
				   wrench_msg->header.stamp, ros::Duration(10));
	transform_t	T;
	_listener.lookupTransform(_sensor_frame, _reference_frame,
	 			  wrench_msg->header.stamp, T);

	wrench_p	wrench(new wrench_t);
	wrench->header = wrench_msg->header;
	wrench->wrench = wrench_msg->wrench;
	wrench->header.frame_id = _sensor_frame;

	tf::Matrix3x3 Rt = T.getBasis();
	tf::Vector3   force  = Rt * _mg + _f_offset;
	tf::Vector3   torque = _r_offset.cross(Rt * _mg) + _m_offset;

	wrench->wrench.force.x  = wrench_msg->wrench.force.x  - force.x();
	wrench->wrench.force.y  = wrench_msg->wrench.force.y  - force.y();
	wrench->wrench.force.z  = wrench_msg->wrench.force.z  - force.z();
	wrench->wrench.torque.x = wrench_msg->wrench.torque.x - torque.x();
	wrench->wrench.torque.y = wrench_msg->wrench.torque.y - torque.y();
	wrench->wrench.torque.z = wrench_msg->wrench.torque.z - torque.z();

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
